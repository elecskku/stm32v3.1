/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *                    [3차] W: UART RX DMA double buffer, R: 8-bit gray + UART TX DMA
  *                    double buffer, DWT stage timing. Search "[3차]" for every change.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18

#define UART_DATA_SIZE       4096U
#define FLASH_PAGE_SIZE      256U
#define FLASH_SECTOR_SIZE    4096U
#define FLASH_TOTAL_SIZE     (4U * 1024U * 1024U)

#define FLASH_CMD_JEDEC_ID       0x9F
#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_READ_STATUS1   0x05
#define FLASH_CMD_PAGE_PROGRAM   0x02
#define FLASH_CMD_READ_DATA      0x03
#define FLASH_CMD_SECTOR_ERASE   0x20
#define FLASH_CMD_CHIP_ERASE     0xC7

uint8_t uart_rx_buffer[UART_DATA_SIZE];

/* [3차] Stage timing, measured with the DWT cycle counter (1 cycle = 10 ns @100 MHz).
 * Accumulated as CPU cycles, converted to milliseconds only when the PC sends 'T'. */
uint64_t cyc_uart_rx = 0;              /* (1) PC -> MCU UART                      */
uint64_t cyc_flash_write = 0;          /* (2) SPI Flash Write                     */
uint64_t cyc_flash_read = 0;           /* (3) SPI Flash Read                      */
uint64_t cyc_gray = 0;                 /* (4) RGB -> Gray                         */
volatile uint64_t cyc_uart_tx = 0;     /* (5) MCU -> PC UART (time the line is busy) */

/* [3차] Optimization switches (set both to 0 for the Baseline measurement)
 * USE_UART_RX_DMA 1 : W - receive the next packet with DMA while the previous one
 *                         is written to Flash (double buffer)
 * USE_UART_TX_DMA 1 : R - send a gray row with DMA while the next row is read and
 *                         converted (double buffer)                                  */
#define USE_UART_RX_DMA    1
#define USE_UART_TX_DMA    1

/* [3차] W double buffer: one whole packet frame = seq(4) + len(2) + data(<=4096) + crc(4) */
#define FRAME_HDR_SIZE     6U
#define FRAME_MAX_SIZE     (FRAME_HDR_SIZE + UART_DATA_SIZE + 4U)
uint8_t rx_frame[2][FRAME_MAX_SIZE];
volatile uint8_t  rx_done = 0;             /* 1 = DMA received the whole frame   */
volatile uint8_t  rx_error = 0;            /* 1 = UART error during DMA receive  */
volatile uint32_t rx_done_cyc = 0;         /* DWT value when the frame completed */

/* [3차] Gray BMP output: 8-bit BMP = header 54 B + gray palette 1024 B + 1 B/pixel rows. */
#define GRAY_HDR_SIZE      54U
#define GRAY_PAL_SIZE      1024U
#define GRAY_DATA_OFFSET   (GRAY_HDR_SIZE + GRAY_PAL_SIZE)   /* 1078 */
#define MAX_COLOR_ROW      6144U    /* 24-bit row incl. padding -> width up to 2048 */
#define MAX_GRAY_ROW       2048U    /* 8-bit row incl. padding  -> width up to 2048 */

uint8_t color_row[MAX_COLOR_ROW];          /* one 24-bit row read from Flash           */
uint8_t gray_row[2][MAX_GRAY_ROW];         /* ping-pong: one is sent by DMA, one is filled */
uint8_t gray_header[GRAY_HDR_SIZE + GRAY_PAL_SIZE];

volatile uint8_t  tx_done = 1;             /* 1 = UART DMA idle, 0 = DMA still sending */
volatile uint32_t tx_start_cyc = 0;        /* DWT value when the current DMA send began */

/* Remember last successfully stored packet for ACK-loss retries. */
uint8_t last_packet_valid = 0;
uint32_t last_packet_seq = 0;
uint16_t last_packet_len = 0;
uint32_t last_packet_crc = 0;


/* File information kept in MCU SRAM.
 * These values are lost if the STM32 is reset or powered off. */
uint32_t stored_file_size = 0;
uint32_t stored_file_crc  = 0;

volatile uint32_t flash_erase_ms = 0;
volatile uint32_t flash_write_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Flash_CS_Low(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void Flash_CS_High(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static void Flash_Read_ID(uint8_t *id)
{
    uint8_t cmd = FLASH_CMD_JEDEC_ID;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);
    Flash_CS_High();
}

static void Flash_WriteEnable(void)
{
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    Flash_CS_High();
}

static uint8_t Flash_ReadStatus(void)
{
    uint8_t cmd = FLASH_CMD_READ_STATUS1;
    uint8_t status = 0;

    Flash_CS_Low();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    Flash_CS_High();

    return status;
}

static HAL_StatusTypeDef Flash_WaitBusy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (Flash_ReadStatus() & 0x01U)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef Flash_SectorErase(uint32_t address)
{
    uint8_t cmd[4];

    if (address >= FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    Flash_WriteEnable();

    cmd[0] = FLASH_CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();
    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }
    Flash_CS_High();

    return Flash_WaitBusy(5000);
}

static HAL_StatusTypeDef Flash_ChipErase(void)
{
    uint8_t cmd = FLASH_CMD_CHIP_ERASE;

    Flash_WriteEnable();

    Flash_CS_Low();
    if (HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }
    Flash_CS_High();

    /* Chip erase may take several seconds. */
    return Flash_WaitBusy(60000);
}

static HAL_StatusTypeDef Flash_PageProgram(uint32_t address,
                                           const uint8_t *data,
                                           uint16_t length)
{
    uint8_t cmd[4];

    if ((length == 0U) ||
        (length > FLASH_PAGE_SIZE) ||
        (address + length > FLASH_TOTAL_SIZE))
    {
        return HAL_ERROR;
    }

    Flash_WriteEnable();

    cmd[0] = FLASH_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();

    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    Flash_CS_High();

    return Flash_WaitBusy(1000);
}

volatile uint32_t flash_debug_page = 0;

static HAL_StatusTypeDef Flash_Write(uint32_t address,
                                     const uint8_t *data,
                                     uint32_t length)
{
    uint32_t offset = 0;

    flash_debug_page = 0;

    if (address + length > FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    while (offset < length)
    {
        uint32_t current_address = address + offset;
        uint32_t page_offset = current_address % FLASH_PAGE_SIZE;
        uint32_t writable = FLASH_PAGE_SIZE - page_offset;
        uint32_t remaining = length - offset;

        uint16_t chunk =
            (uint16_t)((remaining < writable) ? remaining : writable);

        flash_debug_page++;

        if (Flash_PageProgram(current_address,
                              data + offset,
                              chunk) != HAL_OK)
        {
            return HAL_ERROR;
        }

        offset += chunk;
    }

    return HAL_OK;
}


static HAL_StatusTypeDef Flash_Read(uint32_t address,
                                    uint8_t *data,
                                    uint32_t length)
{
    uint8_t cmd[4];

    if (address + length > FLASH_TOTAL_SIZE)
        return HAL_ERROR;

    cmd[0] = FLASH_CMD_READ_DATA;
    cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(address & 0xFFU);

    Flash_CS_Low();

    if (HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    if (HAL_SPI_Receive(&hspi1, data, (uint16_t)length, HAL_MAX_DELAY) != HAL_OK)
    {
        Flash_CS_High();
        return HAL_ERROR;
    }

    Flash_CS_High();
    return HAL_OK;
}

static uint32_t CRC32_Update(uint32_t crc,
                             const uint8_t *data,
                             uint32_t length)
{
    static uint32_t table[256];
    static uint8_t initialized = 0;

    // Lookup Table 최초 1회 생성
    if (!initialized)
    {
        for (uint32_t i = 0; i < 256; i++)
        {
            uint32_t value = i;

            for (uint8_t j = 0; j < 8; j++)
            {
                if (value & 1U)
                    value = (value >> 1) ^ 0xEDB88320U;
                else
                    value >>= 1;
            }

            table[i] = value;
        }

        initialized = 1;
    }

    // Table 기반 CRC32 계산
    for (uint32_t i = 0; i < length; i++)
    {
        uint8_t index = (uint8_t)(crc ^ data[i]);

        crc = (crc >> 8) ^ table[index];
    }

    return crc;
}

static uint16_t ReadU16LE(const uint8_t *data)
{
    return ((uint16_t)data[0]) |
           ((uint16_t)data[1] << 8);
}

static uint32_t ReadU32LE(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/* 24-bit uncompressed BMP layout, required for grayscale readback. */
typedef struct {
    uint32_t pixel_offset;
    uint32_t width;
    uint32_t row_stride;
    uint32_t height;
    uint32_t height_raw;   /* as stored in the file (negative = top-down) */
} BMP_Info;

static uint8_t BMP_ReadInfo(BMP_Info *bmp)
{
    uint8_t h[54];
    uint32_t dib;
    int32_t width, height;
    uint64_t stride, end;
    uint16_t planes, bits;

    if (stored_file_size < sizeof(h) ||
        Flash_Read(0, h, sizeof(h)) != HAL_OK)
        return 0;

    dib = ReadU32LE(h + 14);
    width = (int32_t)ReadU32LE(h + 18);
    height = (int32_t)ReadU32LE(h + 22);
    planes = ReadU16LE(h + 26);
    bits = ReadU16LE(h + 28);
    if (h[0] != 'B' || h[1] != 'M' || dib != 40U ||
        width <= 0 || height == 0 || height == INT32_MIN ||
        planes != 1U || bits != 24U || ReadU32LE(h + 30) != 0U)
        return 0;

    bmp->pixel_offset = ReadU32LE(h + 10);
    bmp->width = (uint32_t)width;
    bmp->height = (height < 0) ? (uint32_t)(-height) : (uint32_t)height;
    bmp->height_raw = (uint32_t)height;
    stride = (((uint64_t)bmp->width * 3U + 3U) / 4U) * 4U;
    end = (uint64_t)bmp->pixel_offset + stride * bmp->height;
    if (bmp->pixel_offset < 54U || stride > UINT32_MAX ||
        end > stored_file_size)
        return 0;
    bmp->row_stride = (uint32_t)stride;
    return 1;
}

/* [3차] DWT cycle counter: start once, then read DWT->CYCCNT before/after each stage. */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Cycles_To_ms(uint64_t cycles)
{
    return (uint32_t)(cycles / (SystemCoreClock / 1000U));
}

/* [3차] One 24-bit row (B,G,R order) -> one 8-bit gray row, padding bytes set to 0.
 * Gray = (77 R + 150 G + 29 B) >> 8   (weights 0.299 / 0.587 / 0.114 scaled by 256) */
static void Gray_ConvertRow(const uint8_t *src, uint8_t *dst,
                            uint32_t width, uint32_t dst_stride)
{
    for (uint32_t x = 0; x < width; x++)
    {
        uint32_t b = src[3U * x];
        uint32_t g = src[3U * x + 1U];
        uint32_t r = src[3U * x + 2U];
        dst[x] = (uint8_t)((77U * r + 150U * g + 29U * b) >> 8);
    }
    for (uint32_t x = width; x < dst_stride; x++)
        dst[x] = 0;
}

/* [3차] 8-bit BMP header (54 B) followed by a 256-entry gray palette (1024 B). */
static void Gray_BuildHeader(uint8_t *out, const BMP_Info *bmp, uint32_t dst_stride)
{
    uint32_t image_size = dst_stride * bmp->height;

    memset(out, 0, GRAY_HDR_SIZE);
    out[0] = 'B';
    out[1] = 'M';
    WriteU32LE(out + 2,  GRAY_DATA_OFFSET + image_size); /* bfSize      */
    WriteU32LE(out + 10, GRAY_DATA_OFFSET);              /* bfOffBits   */
    WriteU32LE(out + 14, 40U);                           /* biSize      */
    WriteU32LE(out + 18, bmp->width);                    /* biWidth     */
    WriteU32LE(out + 22, bmp->height_raw);               /* biHeight    */
    out[26] = 1;                                         /* biPlanes    */
    out[28] = 8;                                         /* biBitCount  */
    WriteU32LE(out + 34, image_size);                    /* biSizeImage */
    WriteU32LE(out + 46, 256U);                          /* biClrUsed   */

    for (uint32_t i = 0; i < 256U; i++)                  /* palette: B=G=R=i */
    {
        out[GRAY_HDR_SIZE + 4U * i]      = (uint8_t)i;
        out[GRAY_HDR_SIZE + 4U * i + 1U] = (uint8_t)i;
        out[GRAY_HDR_SIZE + 4U * i + 2U] = (uint8_t)i;
        out[GRAY_HDR_SIZE + 4U * i + 3U] = 0;
    }
}

/* [3차] Send one gray row.
 * DMA mode : starts the transfer and returns at once; the CPU keeps working.
 *            HAL_UART_TxCpltCallback() sets tx_done = 1 when the last byte has left.
 * Baseline : blocking send, the CPU waits here until the whole row is out. */
static HAL_StatusTypeDef UART_SendRow(uint8_t *row, uint16_t length)
{
#if USE_UART_TX_DMA
    tx_done = 0;
    tx_start_cyc = DWT->CYCCNT;
    if (HAL_UART_Transmit_DMA(&huart2, row, length) != HAL_OK)
    {
        tx_done = 1;
        return HAL_ERROR;
    }
    return HAL_OK;
#else
    uint32_t t = DWT->CYCCNT;
    HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2, row, length, 1000);
    cyc_uart_tx += DWT->CYCCNT - t;
    tx_done = 1;
    return st;
#endif
}

#if USE_UART_RX_DMA
/* [3차] Discard incoming bytes until the line has been silent for 50 ms. */
static void UART_Drain(void)
{
    uint8_t d;
    while (HAL_UART_Receive(&huart2, &d, 1, 50) == HAL_OK) { }
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

/* [3차] Start receiving one packet frame into rx_frame[buf] with DMA. Returns at once. */
static HAL_StatusTypeDef UART_ArmFrame_DMA(uint8_t buf, uint32_t data_len, uint32_t *arm_cyc)
{
    rx_done = 0;
    rx_error = 0;
    *arm_cyc = DWT->CYCCNT;
    return HAL_UART_Receive_DMA(&huart2, rx_frame[buf],
                                (uint16_t)(FRAME_HDR_SIZE + data_len + 4U));
}

/* [3차] Wait until the armed frame is complete. 0 = ok, 1 = UART error, 2 = timeout. */
static uint8_t UART_WaitFrame(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!rx_done && !rx_error)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            (void)HAL_UART_AbortReceive(&huart2);
            return 2;
        }
    }
    return rx_error ? 1 : 0;
}
#endif

static void UART_SendResponse(uint8_t response)
{
    (void)HAL_UART_Transmit(&huart2, &response, 1, 1000);
}

/* A previously accepted packet may be resent if its ACK was lost. */
static void HandleFinalPacketRetry(uint8_t first_byte)
{
    uint8_t header[6], crc_bytes[4];
    uint32_t crc, seq;
    uint16_t len;
    header[0] = first_byte;
    if (HAL_UART_Receive(&huart2, header + 1, 5, 5000) != HAL_OK)
        return;
    seq = ReadU32LE(header);
    len = ReadU16LE(header + 4);
    if (len == 0U || len > UART_DATA_SIZE)
        return;
    if (HAL_UART_Receive(&huart2, uart_rx_buffer, len, 5000) != HAL_OK ||
        HAL_UART_Receive(&huart2, crc_bytes, 4, 5000) != HAL_OK)
        return;
    crc = CRC32_Update(0xFFFFFFFFU, header, 6);
    crc = CRC32_Update(crc, uart_rx_buffer, len) ^ 0xFFFFFFFFU;
    if (last_packet_valid && seq == last_packet_seq &&
        len == last_packet_len && crc == last_packet_crc &&
        ReadU32LE(crc_bytes) == crc)
        UART_SendResponse(ACK);
    else
        UART_SendResponse(NAK);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Keep Flash deselected while idle. */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  DWT_Init();   /* [3차] start the cycle counter used for stage timing */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t cmd;

    if (HAL_UART_Receive(&huart2, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        continue;
    }

    switch (cmd)
    {
        /* I: ACK + JEDEC ID(3B) + Status(1B) */
        case 'I':
        {
            uint8_t id[3];
            uint8_t status;
            uint8_t response = ACK;

            Flash_Read_ID(id);
            status = Flash_ReadStatus();

            HAL_UART_Transmit(&huart2, &response, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, id, 3, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart2, &status, 1, HAL_MAX_DELAY);
            break;
        }

        /* E: Chip erase, then ACK. */
        case 'E':
        {
            uint8_t response;

            if (Flash_ChipErase() == HAL_OK)
            {
                stored_file_size = 0;
                stored_file_crc = 0;
                last_packet_valid = 0;
                response = ACK;
            }
            else
            {
                response = CAN;
            }

            HAL_UART_Transmit(&huart2, &response, 1, HAL_MAX_DELAY);
            break;
        }

        /* W: 8-byte metadata, start ACK, then Stop-and-Wait packets. */
        case 'W':
#if !USE_UART_RX_DMA
        {
            uint8_t metadata[8];
            uint32_t expected_size, expected_crc;
            uint32_t expected_sequence = 0, total_size = 0;
            uint32_t running_crc = 0xFFFFFFFFU;
            uint8_t write_failed = 0;

            last_packet_valid = 0;
            stored_file_size = 0;
            stored_file_crc = 0;
            cyc_uart_rx = cyc_flash_write = 0;   /* [3차] new W = new measurement */

            if (HAL_UART_Receive(&huart2, metadata, 8, 5000) != HAL_OK)
            {
                UART_SendResponse(CAN);
                break;
            }
            expected_size = ReadU32LE(metadata);
            expected_crc = ReadU32LE(metadata + 4);
            if (expected_size == 0U || expected_size > FLASH_TOTAL_SIZE)
            {
                UART_SendResponse(CAN);
                break;
            }
            UART_SendResponse(ACK); /* PC may now send packet #0. */

            while (total_size < expected_size)
            {
                uint8_t header[6], crc_bytes[4];
                uint32_t sequence, received_crc, packet_crc, flash_address;
                uint16_t length;
                uint32_t t;

                /* A header may arrive after the flash operation/ACK. */
                t = DWT->CYCCNT;   /* [3차] (1) MCU ready -> whole frame received */
                if (HAL_UART_Receive(&huart2, header, 6, 5000) != HAL_OK)
                {
                    write_failed = 1;
                    break;
                }
                sequence = ReadU32LE(header);
                length = ReadU16LE(header + 4);
                if (length == 0U || length > UART_DATA_SIZE)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }

                if (HAL_UART_Receive(&huart2, uart_rx_buffer, length, 5000) != HAL_OK ||
                    HAL_UART_Receive(&huart2, crc_bytes, 4, 5000) != HAL_OK)
                {
                    UART_SendResponse(CAN); /* Partial frame: cannot safely resync. */
                    write_failed = 1;
                    break;
                }
                cyc_uart_rx += DWT->CYCCNT - t;

                received_crc = ReadU32LE(crc_bytes);
                packet_crc = CRC32_Update(0xFFFFFFFFU, header, 6);
                packet_crc = CRC32_Update(packet_crc, uart_rx_buffer, length);
                packet_crc ^= 0xFFFFFFFFU;

                if (packet_crc != received_crc)
                {
                    UART_SendResponse(NAK);
                    continue;
                }
                if (last_packet_valid && sequence == last_packet_seq)
                {
                    /* Resend ACK only for the exact previous packet. */
                    if (length == last_packet_len && received_crc == last_packet_crc)
                        UART_SendResponse(ACK);
                    else
                        UART_SendResponse(NAK);
                    continue;
                }
                if (sequence != expected_sequence ||
                    length != ((expected_size - total_size > UART_DATA_SIZE)
                        ? UART_DATA_SIZE : (expected_size - total_size)))
                {
                    UART_SendResponse(NAK);
                    continue;
                }
                flash_address = total_size;

                t = DWT->CYCCNT;   /* [3차] (2) Flash write starts */
                /* [3차] No erase here: E (one chip erase) must be run before W. */
                if (Flash_Write(flash_address, uart_rx_buffer, length) != HAL_OK)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }
                cyc_flash_write += DWT->CYCCNT - t;
                running_crc = CRC32_Update(running_crc, uart_rx_buffer, length);
                total_size += length;
                expected_sequence++;
                last_packet_valid = 1;
                last_packet_seq = sequence;
                last_packet_len = length;
                last_packet_crc = received_crc;

                if (total_size == expected_size)
                {
                    /* Full-file transmitted CRC check; V later checks Flash contents. */
                    if ((running_crc ^ 0xFFFFFFFFU) != expected_crc)
                    {
                        UART_SendResponse(CAN);
                        write_failed = 1;
                        break;
                    }
                    stored_file_size = expected_size;
                    stored_file_crc = expected_crc;
                }
                UART_SendResponse(ACK); /* Flash program and checks have finished. */
            }

            if (write_failed)
            {
                stored_file_size = 0;
                stored_file_crc = 0;
                last_packet_valid = 0;
            }
            break;
        }
#else
        /* [3차] W with UART RX DMA + double buffer (same packet format and replies as above).
         * The ACK is sent as soon as a packet is received and its CRC is OK; the packet is
         * then written to Flash while DMA is already receiving the next one:
         *
         *   DMA : [receive pkt k+1 into buf B.....][receive pkt k+2 into buf A.....]
         *   CPU : [write pkt k (buf A) to Flash]   [write pkt k+1 (buf B)]
         *
         * If a Flash write fails, the reply to the NEXT packet is CAN. The last packet is
         * written before its ACK, so the final ACK still means "whole file stored".      */
        {
            uint8_t metadata[8];
            uint32_t expected_size, expected_crc;
            uint32_t expected_sequence = 0, total_size = 0;
            uint32_t running_crc = 0xFFFFFFFFU;
            uint32_t arm_cyc = 0, t;
            uint32_t pend_addr = 0, pend_len = 0;
            uint8_t *pend_data = 0;
            uint8_t cur = 0, write_failed = 0, flash_error = 0;

            last_packet_valid = 0;
            stored_file_size = 0;
            stored_file_crc = 0;
            cyc_uart_rx = cyc_flash_write = 0;

            if (HAL_UART_Receive(&huart2, metadata, 8, 5000) != HAL_OK)
            {
                UART_SendResponse(CAN);
                break;
            }
            expected_size = ReadU32LE(metadata);
            expected_crc = ReadU32LE(metadata + 4);
            if (expected_size == 0U || expected_size > FLASH_TOTAL_SIZE)
            {
                UART_SendResponse(CAN);
                break;
            }

            /* Arm the receiver BEFORE answering, so no byte of packet #0 is missed. */
            if (UART_ArmFrame_DMA(cur, (expected_size > UART_DATA_SIZE) ?
                                  UART_DATA_SIZE : expected_size, &arm_cyc) != HAL_OK)
            {
                UART_SendResponse(CAN);
                break;
            }
            UART_SendResponse(ACK); /* PC may now send packet #0. */

            while (total_size < expected_size)
            {
                uint8_t *frame, wait_result, reply;
                uint32_t sequence, received_crc, packet_crc, remaining_len;
                uint16_t length;

                /* (2) CPU: write the previous packet while DMA receives this one */
                if (pend_len != 0U)
                {
                    t = DWT->CYCCNT;
                    if (Flash_Write(pend_addr, pend_data, pend_len) != HAL_OK)
                        flash_error = 1;
                    cyc_flash_write += DWT->CYCCNT - t;
                    pend_len = 0;
                }

                /* (1) wait for the frame */
                wait_result = UART_WaitFrame(5000);
                if (wait_result == 2)          /* nothing (complete) arrived */
                {
                    write_failed = 1;
                    break;
                }
                if (flash_error)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }

                remaining_len = expected_size - total_size;
                if (remaining_len > UART_DATA_SIZE) remaining_len = UART_DATA_SIZE;

                if (wait_result == 1)          /* framing / overrun error: resync, resend */
                {
                    UART_Drain();
                    (void)UART_ArmFrame_DMA(cur, remaining_len, &arm_cyc);
                    UART_SendResponse(NAK);
                    continue;
                }
                cyc_uart_rx += rx_done_cyc - arm_cyc;

                frame = rx_frame[cur];
                sequence = ReadU32LE(frame);
                length = ReadU16LE(frame + 4);
                reply = ACK;

                if (length == 0U || length > UART_DATA_SIZE)
                {
                    reply = NAK;
                }
                else
                {
                    received_crc = ReadU32LE(frame + FRAME_HDR_SIZE + length);
                    packet_crc = CRC32_Update(0xFFFFFFFFU, frame, FRAME_HDR_SIZE + length);
                    packet_crc ^= 0xFFFFFFFFU;

                    if (packet_crc != received_crc)
                        reply = NAK;
                    else if (last_packet_valid && sequence == last_packet_seq)
                        /* Resend ACK only for the exact previous packet. */
                        reply = (length == last_packet_len && received_crc == last_packet_crc)
                                ? ACK : NAK;
                    else if (sequence != expected_sequence || length != remaining_len)
                        reply = NAK;
                    else
                        reply = 0;             /* new, valid packet: accept below */
                }

                if (reply != 0U)               /* NAK, or ACK for a repeated packet */
                {
                    (void)UART_ArmFrame_DMA(cur, remaining_len, &arm_cyc);
                    UART_SendResponse(reply);
                    continue;
                }

                /* accept packet */
                running_crc = CRC32_Update(running_crc, frame + FRAME_HDR_SIZE, length);
                last_packet_valid = 1;
                last_packet_seq = sequence;
                last_packet_len = length;
                last_packet_crc = received_crc;
                expected_sequence++;

                if (total_size + length == expected_size)
                {
                    /* Last packet: write it now, check the whole-file CRC, then ACK. */
                    t = DWT->CYCCNT;
                    if (Flash_Write(total_size, frame + FRAME_HDR_SIZE, length) != HAL_OK)
                    {
                        UART_SendResponse(CAN);
                        write_failed = 1;
                        break;
                    }
                    cyc_flash_write += DWT->CYCCNT - t;
                    total_size += length;

                    if ((running_crc ^ 0xFFFFFFFFU) != expected_crc)
                    {
                        UART_SendResponse(CAN);
                        write_failed = 1;
                        break;
                    }
                    stored_file_size = expected_size;
                    stored_file_crc = expected_crc;
                    UART_SendResponse(ACK);
                    break;
                }

                /* Not the last packet: remember it, switch buffers, arm, ACK. */
                pend_addr = total_size;
                pend_data = frame + FRAME_HDR_SIZE;
                pend_len = length;
                total_size += length;

                cur ^= 1U;
                remaining_len = expected_size - total_size;
                if (remaining_len > UART_DATA_SIZE) remaining_len = UART_DATA_SIZE;
                if (UART_ArmFrame_DMA(cur, remaining_len, &arm_cyc) != HAL_OK)
                {
                    UART_SendResponse(CAN);
                    write_failed = 1;
                    break;
                }
                UART_SendResponse(ACK);
            }

            if (write_failed)
            {
                (void)HAL_UART_AbortReceive(&huart2);
                stored_file_size = 0;
                stored_file_crc = 0;
                last_packet_valid = 0;
            }
            break;
        }
#endif

        /* [3차] R: Flash -> row read -> 8-bit Gray -> UART DMA (ping-pong), Flash unchanged.
         * Reply: ACK, size(4B), gray BMP bytes, CRC32 of those bytes(4B).
         * With USE_UART_TX_DMA = 1 the rows overlap like this:
         *
         *   CPU : [read row n+1][gray n+1][wait] [read row n+2][gray n+2][wait] ...
         *   DMA :                         [send row n ..........][send row n+1 ......]
         */
        case 'R':
        {
            BMP_Info bmp;
            uint8_t size_bytes[4], crc_bytes[4];
            uint32_t dst_stride, gray_size, crc = 0xFFFFFFFFU;
            uint32_t y, t;
            uint8_t cur = 0, ok = 1;

            if (stored_file_size == 0U || !BMP_ReadInfo(&bmp))
            {
                UART_SendResponse(NAK);
                break;
            }
            dst_stride = (bmp.width + 3U) & ~3U;
            if (bmp.row_stride > MAX_COLOR_ROW || dst_stride > MAX_GRAY_ROW)
            {
                UART_SendResponse(NAK);   /* image wider than our row buffers */
                break;
            }
            gray_size = GRAY_DATA_OFFSET + dst_stride * bmp.height;

            cyc_flash_read = cyc_gray = 0;
            cyc_uart_tx = 0;

            UART_SendResponse(ACK);
            WriteU32LE(size_bytes, gray_size);
            if (HAL_UART_Transmit(&huart2, size_bytes, 4, 1000) != HAL_OK)
                break;

            /* header + palette (1078 B) */
            Gray_BuildHeader(gray_header, &bmp, dst_stride);
            crc = CRC32_Update(crc, gray_header, sizeof(gray_header));
            t = DWT->CYCCNT;
            if (HAL_UART_Transmit(&huart2, gray_header, sizeof(gray_header), 1000) != HAL_OK)
                break;
            cyc_uart_tx += DWT->CYCCNT - t;

            /* pixel rows, in the same order they are stored in the file */
            for (y = 0; y < bmp.height; y++)
            {
                t = DWT->CYCCNT;
                if (Flash_Read(bmp.pixel_offset + y * bmp.row_stride,
                               color_row, bmp.row_stride) != HAL_OK)
                {
                    ok = 0;
                    break;
                }
                cyc_flash_read += DWT->CYCCNT - t;

                t = DWT->CYCCNT;
                Gray_ConvertRow(color_row, gray_row[cur], bmp.width, dst_stride);
                cyc_gray += DWT->CYCCNT - t;

                crc = CRC32_Update(crc, gray_row[cur], dst_stride);

                while (!tx_done) { }          /* previous row must have left the UART */
                if (UART_SendRow(gray_row[cur], (uint16_t)dst_stride) != HAL_OK)
                {
                    ok = 0;
                    break;
                }
                cur ^= 1U;                    /* fill the other buffer next time */
            }
            while (!tx_done) { }              /* last row finished */

            if (ok)
            {
                WriteU32LE(crc_bytes, crc ^ 0xFFFFFFFFU);
                (void)HAL_UART_Transmit(&huart2, crc_bytes, 4, 1000);
            }
            break;
        }

        /* V: stored CRC(4B) + CRC recalculated from Flash(4B) */
        case 'V':
        {
            uint32_t calculated_crc = 0xFFFFFFFFU;
            uint32_t offset = 0;
            uint8_t result[8];

            if (stored_file_size == 0U)
            {
                WriteU32LE(&result[0], 0U);
                WriteU32LE(&result[4], 0xFFFFFFFFU);
                HAL_UART_Transmit(&huart2, result, 8, HAL_MAX_DELAY);
                break;
            }

            while (offset < stored_file_size)
            {
                uint32_t remaining = stored_file_size - offset;
                uint32_t chunk = (remaining > UART_DATA_SIZE)
                                 ? UART_DATA_SIZE
                                 : remaining;

                if (Flash_Read(offset, uart_rx_buffer, chunk) != HAL_OK)
                {
                    calculated_crc = 0;
                    break;
                }

                calculated_crc = CRC32_Update(calculated_crc,
                                              uart_rx_buffer,
                                              chunk);
                offset += chunk;
            }

            if (offset == stored_file_size)
                calculated_crc ^= 0xFFFFFFFFU;

            WriteU32LE(&result[0], stored_file_crc);
            WriteU32LE(&result[4], calculated_crc);

            HAL_UART_Transmit(&huart2, result, 8, HAL_MAX_DELAY);
            break;
        }

        /* T: ACK + 5 little-endian uint32_t durations in milliseconds. */
        case 'T':
        {
            uint8_t data[20];
            WriteU32LE(data + 0,  Cycles_To_ms(cyc_uart_rx));
            WriteU32LE(data + 4,  Cycles_To_ms(cyc_flash_write));
            WriteU32LE(data + 8,  Cycles_To_ms(cyc_flash_read));
            WriteU32LE(data + 12, Cycles_To_ms(cyc_gray));
            WriteU32LE(data + 16, Cycles_To_ms(cyc_uart_tx));
            UART_SendResponse(ACK);
            (void)HAL_UART_Transmit(&huart2, data, sizeof(data), 1000);
            break;
        }

        default:
        {
            /* Packet retransmission after final ACK loss has no command framing.
             * Detect only if the first byte matches the final sequence LSB.
             * For a production protocol add an explicit packet/command delimiter. */
            if (last_packet_valid && stored_file_size != 0U &&
                cmd == (uint8_t)(last_packet_seq & 0xFFU))
                HandleFinalPacketRetry(cmd);
            else
                UART_SendResponse(NAK);
            break;
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* [3차] Called from the UART interrupt when a DMA receive has filled the whole frame. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        rx_done_cyc = DWT->CYCCNT;
        rx_done = 1;
    }
}

/* [3차] Called on UART errors (overrun, framing, noise) during a DMA receive. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
        rx_error = 1;
}

/* [3차] Called from the UART interrupt when a DMA send has completely finished. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        cyc_uart_tx += DWT->CYCCNT - tx_start_cyc;
        tx_done = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
