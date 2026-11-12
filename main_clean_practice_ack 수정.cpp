#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <windows.h>
#include <cstdint>
#include <iomanip>
#include <iterator>   // [3차] istreambuf_iterator (gray check)

constexpr DWORD BAUD_RATE = 921600;

constexpr uint8_t ACK = 0x06;
constexpr uint8_t NAK = 0x15;
constexpr uint8_t CAN = 0x18;

constexpr size_t DATA_SIZE = 4096;

constexpr size_t MAX_FILE_SIZE = 4u * 1024u * 1024u;

constexpr int MAX_RETRY = 3;

// [3차] Write 중 진행률을 몇 패킷마다 화면에 찍을지. 화면 출력도 시간이 걸리므로
//       1 = 원래처럼 매 패킷, 50 = 50패킷마다(+마지막). 두 값으로 재서 비교해 본다.
constexpr uint32_t PROGRESS_EVERY = 50;

// [3차] ms 단위 정밀 시계 (GetTickCount64는 약 16 ms 단위라 패킷 하나를 재기엔 거칠다)
static double nowMs()
{
    static LARGE_INTEGER freq;   // static -> starts at 0
    LARGE_INTEGER c;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&c);
    return static_cast<double>(c.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
}

// 입력 데이터의 표준 CRC32를 계산한다.
uint32_t crc32(const uint8_t* data, size_t length)
{

    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++)
    {

        crc ^= data[i];

        for (int j = 0; j < 8; j++)
        {

            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;

            else
                crc >>= 1;
        }

    }

    return crc ^ 0xFFFFFFFF;
}

// Windows COM 포트를 열고 핸들을 반환한다.
HANDLE openSerialPort(const char* portName)
{

    HANDLE hSerial = CreateFileA(
        portName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    return hSerial;
}

// UART 통신 설정(921600bps, 8-N-1)과 타임아웃을 적용한다.
bool configureSerialPort(HANDLE hSerial)
{

    DCB dcb = {0};

    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(hSerial, &dcb))
        return false;

    dcb.BaudRate = BAUD_RATE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    dcb.fInX = FALSE;
    dcb.fOutX = FALSE;

    if (!SetCommState(hSerial, &dcb))
        return false;

    COMMTIMEOUTS timeouts = {0};

    timeouts.ReadIntervalTimeout = MAXDWORD;

    timeouts.ReadTotalTimeoutConstant = 100;

    timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;

    timeouts.WriteTotalTimeoutConstant = 60000;

    timeouts.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(hSerial, &timeouts))
        return false;

    PurgeComm(
        hSerial,
        PURGE_RXCLEAR |
        PURGE_TXCLEAR
    );

    return true;
}

// 지정한 데이터를 모두 Windows 시리얼 포트에 기록한다.
bool writeAll(
    HANDLE hSerial,
    const uint8_t* data,
    size_t length)
{

    size_t totalWritten = 0;

    while (totalWritten < length)
    {

        DWORD written = 0;

        if (!WriteFile(
            hSerial,

            data + totalWritten,

            static_cast<DWORD>(length - totalWritten),

            &written,
            NULL))
        {

            return false;
        }

        if (written == 0)
            return false;

        totalWritten += written;
    }

    return true;
}

// 요청한 길이만큼 수신하되 유휴 타임아웃을 적용한다.
bool readAll(
    HANDLE hSerial,

    uint8_t* data,

    size_t length,
    ULONGLONG idleTimeoutMs = 5000
    )
{

    size_t totalRead = 0;

    ULONGLONG lastReceiveTime = GetTickCount64();

    const DWORD READ_CHUNK_SIZE = 16384;

    while (totalRead < length)
    {

        DWORD bytesRead = 0;
        size_t remaining = length - totalRead;
        DWORD requestSize = static_cast<DWORD>(
            remaining > READ_CHUNK_SIZE
                ? READ_CHUNK_SIZE
                : remaining
        );
        if (!ReadFile(
                hSerial,

                data + totalRead,
                requestSize,

                &bytesRead,
                NULL))
        {

            std::cerr << "UART ReadFile failed. Windows error: "
                      << GetLastError() << "\n";
            return false;
        }

        if (bytesRead > 0)
        {
            totalRead += bytesRead;

            lastReceiveTime = GetTickCount64();
        }

        else
        {
            if (GetTickCount64() - lastReceiveTime
                >= idleTimeoutMs)
            {
                std::cerr << "UART receive timeout ("
                          << idleTimeoutMs << " ms).\n";

                 return false;
            }
        }
    }

    return true;
}

// STM32에 1바이트 명령을 전송한다.
bool sendCommand(
    HANDLE hSerial,

    char command)
{

    uint8_t cmd = static_cast<uint8_t>(command);

    return writeAll(
        hSerial,
        &cmd,
        1
    );
}

// STM32의 1바이트 응답(ACK/NAK/CAN)을 수신한다.
uint8_t receiveResponse(HANDLE hSerial, ULONGLONG idleTimeoutMs = 5000)
{

    uint8_t response = 0;

    if (!readAll(hSerial, &response, 1, idleTimeoutMs))
    {

        std::cout << "Response receive failed.\n";
        return 0;
    }

    if (response != ACK)
    {
        std::cout << "STM32 response: 0x"
                  << std::hex
                  << static_cast<int>(response)
                  << std::dec
                  << "\n";
    }

    return response;
}

// 16비트 값을 Little-endian 바이트 순서로 추가한다.
void appendU16(
    std::vector<uint8_t>& packet,
    uint16_t value)
{
    packet.push_back(
        value & 0xFF
    );

    packet.push_back(
        (value >> 8) & 0xFF
    );
}

// 32비트 값을 Little-endian 바이트 순서로 추가한다.
void appendU32(
    std::vector<uint8_t>& packet,
    uint32_t value)
{
    packet.push_back(
        value & 0xFF
    );

    packet.push_back(
        (value >> 8) & 0xFF
    );

    packet.push_back(
        (value >> 16) & 0xFF
    );

    packet.push_back(
        (value >> 24) & 0xFF
    );
}

// Little-endian 4바이트를 32비트 정수로 복원한다.
uint32_t readU32LE(
    const uint8_t* data)
{
    return
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8) |
        (static_cast<uint32_t>(data[2]) << 16) |
        (static_cast<uint32_t>(data[3]) << 24);
}

// Sequence, Length, Payload, CRC32로 전송 패킷을 생성한다.
std::vector<uint8_t> makePacket(
    uint32_t sequence,
    const uint8_t* data,
    uint16_t length)
{

    std::vector<uint8_t> packet;

    packet.reserve(
        4 + 2 + length + 4
    );

    appendU32(
        packet,
        sequence
    );

    appendU16(
        packet,
        length
    );

    packet.insert(

        packet.end(),

        data,

        data + length
    );

    uint32_t crc = crc32(
        packet.data(),
        packet.size()
    );

    appendU32(
        packet,
        crc
    );

    return packet;
}

// Flash JEDEC ID와 상태 레지스터를 조회한다.
bool commandID(HANDLE hSerial)
{

    std::cout << "[I] Request Flash ID\n";

    if (!sendCommand(
        hSerial,
        'I'))
    {
        return false;
    }

    uint8_t response;

    if (!readAll(
        hSerial,
        &response,
        1))
    {
        return false;
    }

    if (response != ACK)
    {
        std::cout
            << "ID request failed. Response = 0x"
            << std::hex
            << static_cast<int>(response)
            << std::dec
            << "\n";

        return false;
    }

    uint8_t id[3];
    uint8_t status;

    if (!readAll(
        hSerial,

        id,
        3))
    {
        return false;
    }

    if (!readAll(
        hSerial,
        &status,
        1))
    {
        return false;
    }

    std::cout
        << "Flash ID: "
        << std::hex
        << std::setfill('0');

    for (int i = 0; i < 3; i++)
    {
        std::cout
            << "0x"
            << std::setw(2)
            << static_cast<int>(id[i])
            << " ";
    }

    std::cout
        << "\nStatus: 0x"
        << std::setw(2)
        << static_cast<int>(status)
        << std::dec
        << "\n";

    return true;
}

// 파일을 4096B 단위 패킷으로 나눠 Flash로 전송한다.
bool commandWrite(
    HANDLE hSerial,
    const std::string& fileName)
{

    std::ifstream file(
        fileName,

        std::ios::binary
    );

    if (!file)
    {
        std::cout
            << "Failed to open file: "
            << fileName
            << "\n";

        return false;
    }

    file.seekg(0, std::ios::end);

    std::streamoff rawFileSize =
        static_cast<std::streamoff>(file.tellg());

    if (rawFileSize <= 0 ||
        static_cast<uint64_t>(rawFileSize) > MAX_FILE_SIZE)
    {
        std::cout << "Invalid input file size.\n";
        return false;
    }

    size_t fileSize =
        static_cast<size_t>(rawFileSize);

    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> fileData(
        fileSize
    );

    if (!file.read(
        reinterpret_cast<char*>(

            fileData.data()
        ),
        fileSize))
    {
        std::cout
            << "Failed to read file.\n";

        return false;
    }

    std::cout
        << "[W] File: "
        << fileName
        << "\n";

    std::cout
        << "File size: "
        << fileSize
        << " bytes\n";

    uint32_t fileCRC =
        crc32(
            fileData.data(),
            fileData.size()
        );

    std::vector<uint8_t> metadata;

    metadata.reserve(8);

    appendU32(
        metadata,
        static_cast<uint32_t>(fileSize)
    );

    appendU32(
        metadata,
        fileCRC
    );

    ULONGLONG startTime = GetTickCount64();

    if (!sendCommand(
        hSerial,
        'W'))
    {
        return false;
    }

    if (!writeAll(
        hSerial,
        metadata.data(),
        metadata.size()))
    {
        std::cout << "Failed to send metadata.\n";
        return false;
    }

    uint8_t response = receiveResponse(hSerial);

    if (response != ACK)
    {
        std::cout << "Write initialization failed.\n";
        return false;
    }

    size_t totalPackets =
        (fileSize + DATA_SIZE - 1)
        / DATA_SIZE;

    size_t offset = 0;

    // [3차] Write 시간을 PC 쪽에서 나눠 본다: 패킷 만들기 / 보내기 / ACK 기다리기 / 화면 출력
    double pcMakeMs = 0, pcSendMs = 0, pcAckWaitMs = 0, pcPrintMs = 0, tp = 0;

    for (
        uint32_t sequence = 0;
        sequence < totalPackets;
        sequence++)
    {

        size_t remaining =
            fileSize - offset;

        uint16_t length =
            static_cast<uint16_t>(
                remaining > DATA_SIZE
                ? DATA_SIZE
                : remaining
            );

        tp = nowMs();   // [3차]
        std::vector<uint8_t> packet =
            makePacket(
                sequence,
                fileData.data() + offset,
                length
            );
        pcMakeMs += nowMs() - tp;   // [3차]

        bool success = false;

        for (
            int retry = 0;
            retry < MAX_RETRY;
            retry++)
        {

            tp = nowMs();   // [3차]
            if (!writeAll(
                hSerial,
                packet.data(),
                packet.size()))
            {
                return false;
            }
            pcSendMs += nowMs() - tp;   // [3차]

            tp = nowMs();   // [3차]
            uint8_t response =
                receiveResponse(
                    hSerial
                );
            pcAckWaitMs += nowMs() - tp;   // [3차]

            if (response == ACK)
            {
                success = true;
                break;
            }

            else if (response == NAK)
            {
                std::cout
                    << "Packet "
                    << sequence
                    << " NAK (attempt "
                    << retry + 1
                    << " / "
                    << MAX_RETRY
                    << ")\n";
            }

            else if (response == CAN)
            {
                std::cout
                    << "Flash error. Transfer canceled.\n";

                return false;
            }

            else if (response == 0)
            {
                std::cout
                    << "Packet "
                    << sequence
                    << " response not received (attempt "
                    << retry + 1
                    << " / "
                    << MAX_RETRY
                    << ")\n";
            }

            else
            {
                std::cout
                    << "Unexpected response. Transfer stopped.\n";
                return false;
            }
        }

        if (!success)
        {
            std::cout
                << "Packet "
                << sequence
                << " failed after retries.\n";

            return false;
        }

        offset += length;

        double progress =
            (static_cast<double>(
                offset
            ) / fileSize)
            * 100.0;

        // [3차] 진행률은 PROGRESS_EVERY 패킷마다 + 마지막 패킷에서만 찍는다.
        if ((sequence + 1) % PROGRESS_EVERY == 0 || sequence + 1 == totalPackets)
        {
            tp = nowMs();
            std::cout
                << "\rPacket "
                << sequence + 1
                << " / "
                << totalPackets
                << "  "
                << std::fixed
                << std::setprecision(1)
                << progress
                << "%"
                << std::flush;
            pcPrintMs += nowMs() - tp;
        }
    }

    ULONGLONG endTime = GetTickCount64();

    double elapsedTime =
        (endTime - startTime) / 1000.0;

    double transferSpeed =
        (fileSize / 1024.0) / elapsedTime;

    std::cout
        << "\nWrite complete.\n";
    std::cout
        << "Transfer time: "
        << std::fixed
        << std::setprecision(2)
        << elapsedTime
        << " seconds\n";

    std::cout
        << "Average speed: "
        << transferSpeed
        << " KiB/s\n";

    // [3차] PC 쪽에서 본 Write 시간 나누기 (ms)
    //   send     : WriteFile이 패킷을 넘기는 데 걸린 시간
    //   ack wait : 보내기가 끝난 뒤 ACK가 도착할 때까지 (선으로 나가는 시간 + 보드 처리 + USB 왕복)
    std::cout << std::setprecision(0)
              << "  PC make packets : " << pcMakeMs    << " ms\n"
              << "  PC send (Write) : " << pcSendMs    << " ms\n"
              << "  PC wait for ACK : " << pcAckWaitMs << " ms\n"
              << "  PC print        : " << pcPrintMs   << " ms  (every "
              << PROGRESS_EVERY << " packets)\n";

    return true;
}

// STM32가 보낸 BMP를 수신하고 CRC32 검증 후 저장한다.
bool commandRead(
    HANDLE hSerial,
    const std::string& outputFile)
{
    std::cout
        << "[R] Restore file\n";

    if (!sendCommand(
        hSerial,
        'R'))
    {
        return false;
    }

    uint8_t response =
        receiveResponse(
            hSerial
        );

    if (response != ACK)
    {
        std::cout
            << "Read request rejected.\n";

        return false;
    }

    uint8_t sizeBytes[4];

    if (!readAll(
        hSerial,
        sizeBytes,
        4))
    {
        return false;
    }

    uint32_t fileSize =
        readU32LE(sizeBytes);

    if (fileSize == 0 || fileSize > MAX_FILE_SIZE)
    {
        std::cout << "Invalid received file size: "
                  << fileSize << " bytes\n";

        return false;
    }

    std::cout
        << "File size: "
        << fileSize
        << " bytes\n";

    std::vector<uint8_t> data(fileSize);

    if (!readAll(
        hSerial,
        data.data(),
        fileSize))
    {
        std::cout
            << "Failed while receiving file.\n";

        return false;
    }

    uint8_t crcBytes[4];

    if (!readAll(
        hSerial,
        crcBytes,
        4))
    {
        return false;
    }

    uint32_t stm32CRC =
        readU32LE(
            crcBytes
        );

    uint32_t pcCRC =
        crc32(
            data.data(),
            data.size()
        );

    std::cout
        << "STM32 CRC32: 0x"
        << std::hex
        << stm32CRC
        << "\n";

    std::cout
        << "PC CRC32   : 0x"
        << pcCRC
        << std::dec
        << "\n";

    if (stm32CRC != pcCRC)
    {
        std::cout
            << "CRC mismatch.\n";

        return false;
    }

    std::ofstream file(
        outputFile,
        std::ios::binary
    );

    if (!file)
    {
        std::cout
            << "Failed to create output file.\n";

        return false;
    }

    file.write(
        reinterpret_cast<const char*>(
            data.data()
        ),
    static_cast<std::streamsize>(data.size())
    );

    file.close();

    if (!file)
    {
        std::cout << "Failed to save output file.\n";
        return false;
    }

    std::cout
        << "Restored file saved: "
        << outputFile
        << "\n";

    return true;
}

// Flash의 기준 CRC32와 재계산 CRC32를 비교한다.
bool commandVerify(
    HANDLE hSerial)
{
    std::cout
        << "[V] Verify Flash data\n";

    ULONGLONG verifyStartTime = GetTickCount64();

    if (!sendCommand(
        hSerial,
        'V'))
    {
        return false;
    }

    uint8_t crcData[8];

    if (!readAll(
        hSerial,
        crcData,
        8,
        30000))
    {
        return false;
    }

    uint32_t storedCRC =
        readU32LE(
            crcData
        );

    uint32_t calculatedCRC =
        readU32LE(
            crcData + 4
        );

    std::cout
        << "Stored CRC     : 0x"
        << std::hex
        << storedCRC
        << "\n";

    std::cout
        << "Calculated CRC : 0x"
        << calculatedCRC
        << std::dec
        << "\n";

    ULONGLONG verifyEndTime = GetTickCount64();

    double verifyElapsedTime =
        (verifyEndTime - verifyStartTime) / 1000.0;

    std::cout
        << "Verify time: "
        << std::fixed
        << std::setprecision(2)
        << verifyElapsedTime
        << " seconds\n";

    if (storedCRC == calculatedCRC)
    {
        std::cout
            << "VERIFY PASS\n";

        return true;
    }

    std::cout
        << "VERIFY FAIL\n";

    return false;
}

// Flash 전체 삭제를 요청하고 완료 응답을 기다린다.
bool commandErase(
    HANDLE hSerial)
{
    std::cout
        << "[E] Flash erase requested...\n";

    if (!sendCommand(
        hSerial,
        'E'))
    {
        return false;
    }

    uint8_t response =
        receiveResponse(
            hSerial,
            65000
        );

    if (response == ACK)
    {
        std::cout
            << "Erase complete.\n";

        return true;
    }

    std::cout
        << "Erase failed.\n";

    return false;
}

// [3차] 변환 결과 확인: 원본 컬러 BMP로 PC에서 "정답 흑백 BMP"를 직접 만들어
//       STM32가 보낸 흑백 BMP와 한 바이트씩 비교한다.
//       8-bit 흑백 (77R + 150G + 29B) >> 8, 헤더 54 B + 팔레트 1024 B 와
//       24-bit 흑백 (114B + 587G + 299R + 500) / 1000 (팀 원본 코드) 둘 다 확인할 수 있다.
static std::vector<uint8_t> readWholeFile(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)),
                                std::istreambuf_iterator<char>());
}

static void putU32LE(std::vector<uint8_t>& d, size_t pos, uint32_t v)
{
    d[pos] = v & 0xFF;          d[pos + 1] = (v >> 8) & 0xFF;
    d[pos + 2] = (v >> 16) & 0xFF; d[pos + 3] = (v >> 24) & 0xFF;
}

bool commandCheckGray(
    const std::string& colorFile,
    const std::string& grayFile)
{
    std::cout << "[C] Check gray result\n";

    std::vector<uint8_t> color = readWholeFile(colorFile);
    std::vector<uint8_t> gray  = readWholeFile(grayFile);

    if (color.size() < 54 || color[0] != 'B' || color[1] != 'M' || color[28] != 24)
    {
        std::cout << "Original is not a 24-bit BMP.\n";
        return false;
    }
    if (gray.size() < 54 || gray[0] != 'B' || gray[1] != 'M')
    {
        std::cout << "Gray file is not a BMP.\n";
        return false;
    }

    uint32_t width   = readU32LE(&color[18]);
    int32_t  hRaw    = static_cast<int32_t>(readU32LE(&color[22]));
    uint32_t height  = static_cast<uint32_t>(hRaw < 0 ? -hRaw : hRaw);
    uint32_t dataOff = readU32LE(&color[10]);
    uint32_t sStride = (width * 3 + 3) / 4 * 4;

    if (color.size() < static_cast<size_t>(dataOff) + static_cast<size_t>(sStride) * height)
    {
        std::cout << "Original BMP is shorter than its header says.\n";
        return false;
    }

    std::vector<uint8_t> expected;
    int grayBits = gray[28];

    if (grayBits == 8)
    {
        uint32_t dStride = (width + 3) / 4 * 4;
        expected.assign(1078 + static_cast<size_t>(dStride) * height, 0);
        expected[0] = 'B'; expected[1] = 'M';
        putU32LE(expected, 2, 1078 + dStride * height);
        putU32LE(expected, 10, 1078);
        putU32LE(expected, 14, 40);
        putU32LE(expected, 18, width);
        putU32LE(expected, 22, static_cast<uint32_t>(hRaw));
        expected[26] = 1;
        expected[28] = 8;
        putU32LE(expected, 34, dStride * height);
        putU32LE(expected, 46, 256);
        for (int i = 0; i < 256; i++)
        {
            expected[54 + 4 * i]     = static_cast<uint8_t>(i);
            expected[54 + 4 * i + 1] = static_cast<uint8_t>(i);
            expected[54 + 4 * i + 2] = static_cast<uint8_t>(i);
        }
        for (uint32_t y = 0; y < height; y++)
        {
            const uint8_t* s = &color[dataOff + static_cast<size_t>(y) * sStride];
            uint8_t* d = &expected[1078 + static_cast<size_t>(y) * dStride];
            for (uint32_t x = 0; x < width; x++)
                d[x] = static_cast<uint8_t>((77 * s[3 * x + 2] + 150 * s[3 * x + 1] + 29 * s[3 * x]) >> 8);
        }
    }
    else if (grayBits == 24)
    {
        expected = color;                       // header and padding unchanged
        for (uint32_t y = 0; y < height; y++)
        {
            uint8_t* p = &expected[dataOff + static_cast<size_t>(y) * sStride];
            for (uint32_t x = 0; x < width; x++)
            {
                uint8_t g = static_cast<uint8_t>(
                    (114u * p[3 * x] + 587u * p[3 * x + 1] + 299u * p[3 * x + 2] + 500u) / 1000u);
                p[3 * x] = p[3 * x + 1] = p[3 * x + 2] = g;
            }
        }
    }
    else
    {
        std::cout << "Unsupported gray BMP bit count: " << grayBits << "\n";
        return false;
    }

    size_t n = expected.size() < gray.size() ? expected.size() : gray.size();
    size_t diff = (expected.size() > gray.size() ? expected.size() - gray.size()
                                                 : gray.size() - expected.size());
    for (size_t i = 0; i < n; i++)
        if (expected[i] != gray[i]) diff++;

    std::cout << "Gray BMP       : " << grayBits << "-bit, "
              << width << " x " << height << "\n";
    std::cout << "Expected size  : " << expected.size() << " bytes\n";
    std::cout << "Received size  : " << gray.size() << " bytes\n";
    std::cout << "Different bytes: " << diff << "\n";
    std::cout << (diff == 0 ? "GRAY CHECK PASS\n" : "GRAY CHECK FAIL\n");
    return diff == 0;
}

// CLI 명령을 실행하고 Demo 전체 흐름을 관리한다.
int main(
    int argc,
    char* argv[])
{

    // [3차] COM 포트를 실행할 때 지정한다. 예) uart_tool.exe COM8 demo in.bmp out.bmp
    //       첫 인수가 COM으로 시작하지 않으면 예전처럼 COM3을 쓴다.
    std::string portName = "COM3";
    if (argc >= 2 && std::string(argv[1]).rfind("COM", 0) == 0)
    {
        portName = argv[1];
        argc -= 1;
        argv += 1;
    }
    std::string portPath = "\\\\.\\" + portName;   // COM10 이상도 열 수 있게

    HANDLE hSerial =
        openSerialPort(
            portPath.c_str()
        );

    if (
        hSerial ==
        INVALID_HANDLE_VALUE)
    {
        std::cout
            << "Failed to open " << portName << ".\n";

        return 1;
    }

    if (!configureSerialPort(
        hSerial))
    {
        std::cout
            << "Failed to configure " << portName << ".\n";

        CloseHandle(
            hSerial
        );

        return 1;
    }

    std::cout
        << portName << " opened.\n";

std::cout << "UART: "
          << BAUD_RATE
          << " bps, 8-N-1\n";

    if (argc < 2)
    {
        std::cout
            << "\nUsage:\n"
            << "  uart_tool.exe id\n"
            << "  uart_tool.exe write file.bmp\n"
            << "  uart_tool.exe read output.bmp\n"
            << "  uart_tool.exe verify\n"
            << "  uart_tool.exe erase\n"
            << "  uart_tool.exe demo input.bmp output.bmp\n"
            << "  uart_tool.exe check input.bmp gray.bmp      (PC only, no board)\n"
            << "  (put the port first if it is not COM3, e.g. uart_tool.exe COM8 demo a.bmp b.bmp)\n";

        CloseHandle(
            hSerial
        );

        return 0;
    }

    std::string command =
        argv[1];

    bool result = false;

    if (
        command == "id")
    {
        result =
            commandID(
                hSerial
            );
    }

    else if (
        command == "write" &&
        argc >= 3)
    {
        result =
            commandWrite(
                hSerial,
                argv[2]
            );
    }

    else if (
        command == "read" &&
        argc >= 3)
    {
        result =
            commandRead(
                hSerial,
                argv[2]
            );
    }

    else if (
        command == "verify")
    {
        result =
            commandVerify(
                hSerial
            );
    }

    else if (
        command == "erase")
    {
        result =
            commandErase(
                hSerial
            );
    }

    else if (
        command == "check" &&
        argc >= 4)
    {
        result = commandCheckGray(argv[2], argv[3]);   // [3차]
    }

    else if (
        command == "demo" &&
        argc >= 4)
    {

        ULONGLONG demoStartTime = GetTickCount64();
        std::cout
            << "\n=== DEMO START ===\n";

        // [3차] 명령마다 PC에서 걸린 시간을 잰다 (ms).
        ULONGLONG tEraseMs = 0, tWriteMs = 0, tReadMs = 0, tVerifyMs = 0, t0 = 0;

        result =
            commandID(
                hSerial
            );

        if (result)
        {
            t0 = GetTickCount64();
            result =
                commandErase(
                    hSerial
                );
            tEraseMs = GetTickCount64() - t0;
        }

        if (result)
        {
            t0 = GetTickCount64();
            result =
                commandWrite(
                    hSerial,
                    argv[2]
                );
            tWriteMs = GetTickCount64() - t0;
        }

        if (result)
        {
            t0 = GetTickCount64();
            result =
                commandRead(
                    hSerial,
                    argv[3]
                );
            tReadMs = GetTickCount64() - t0;
        }

        // [3차] 변환 결과 확인 (PC에서 정답과 바이트 비교, 시간 측정에는 넣지 않음)
        if (result)
            result =
                commandCheckGray(
                    argv[2],
                    argv[3]
                );

        if (result)
        {
            t0 = GetTickCount64();
            result =
                commandVerify(
                    hSerial
                );
            tVerifyMs = GetTickCount64() - t0;
        }

        if (result)
        {
            uint8_t response;
            uint8_t timing[20];

            if (!sendCommand(hSerial, 'T') ||
                !readAll(hSerial, &response, 1) ||
                response != ACK ||
                !readAll(hSerial, timing, 20))
            {
                std::cout << "Timing request failed.\n";
                result = false;
            }
            else
            {
                std::cout << "\n=== TIMING RESULTS (measured on STM32) ===\n";

                const char* names[5] = {
                    "PC -> MCU UART ",
                    "SPI Flash Write",
                    "SPI Flash Read ",
                    "RGB -> Gray    ",
                    "MCU -> PC UART "
                };

                uint32_t stageSum = 0;   // [3차]
                for (int i = 0; i < 5; i++)
                {
                    uint32_t v = readU32LE(timing + i * 4);
                    stageSum += v;
                    std::cout << names[i] << ": "
                              << v
                              << " ms\n";
                }

                // [3차] 전체 수행시간 = W 시작 ~ 흑백 BMP 수신 완료 (PC 측정)
                ULONGLONG totalMs = tWriteMs + tReadMs;
                std::cout << "Sum of 5 stages: " << stageSum << " ms\n";
                std::cout << "\n=== TIMING RESULTS (measured on PC) ===\n";
                std::cout << "Erase  (E, before W)   : " << tEraseMs  << " ms\n";
                std::cout << "Write  (W)             : " << tWriteMs  << " ms\n";
                std::cout << "Read   (R, gray BMP)   : " << tReadMs   << " ms\n";
                std::cout << "Verify (V, not counted): " << tVerifyMs << " ms\n";
                std::cout << "TOTAL  (W start -> gray BMP received): " << totalMs << " ms\n";
                if (totalMs >= stageSum)
                    std::cout << "Other  (TOTAL - sum of 5 stages)    : " << (totalMs - stageSum) << " ms\n";
                else
                    std::cout << "Sum of 5 stages > TOTAL: DMA makes stages overlap in time.\n";
            }
        }

        ULONGLONG demoEndTime = GetTickCount64();

        double demoElapsedTime =
            (demoEndTime - demoStartTime) / 1000.0;

        if (result)
        {
            std::cout << "\n=== DEMO PASS ===\n";
        }
        else
        {
            std::cout << "\n=== DEMO FAIL ===\n";
        }

        std::cout
            << "Total Demo Time: "
            << std::fixed
            << std::setprecision(2)
            << demoElapsedTime
            << " seconds\n";
    }

    else
    {
        std::cout
            << "Unknown command.\n";
    }

    CloseHandle(
        hSerial
    );

    return result ? 0 : 1;
}
