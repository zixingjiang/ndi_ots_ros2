//----------------------------------------------------------------------------
//
//  Copyright (C) 2017, Northern Digital Inc. All rights reserved.
//
//  All Northern Digital Inc. ("NDI") Media and/or Sample Code and/or Sample Code
//  Documentation (collectively referred to as "Sample Code") is licensed and provided "as
//  is" without warranty of any kind. The licensee, by use of the Sample Code, warrants to
//  NDI that the Sample Code is fit for the use and purpose for which the licensee intends to
//  use the Sample Code. NDI makes no warranties, express or implied, that the functions
//  contained in the Sample Code will meet the licensee's requirements or that the operation
//  of the programs contained therein will be error free. This warranty as expressed herein is
//  exclusive and NDI expressly disclaims any and all express and/or implied, in fact or in
//  law, warranties, representations, and conditions of every kind pertaining in any way to
//  the Sample Code licensed and provided by NDI hereunder, including without limitation,
//  each warranty and/or condition of quality, merchantability, description, operation,
//  adequacy, suitability, fitness for particular purpose, title, interference with use or
//  enjoyment, and/or non infringement, whether express or implied by statute, common law,
//  usage of trade, course of dealing, custom, or otherwise. No NDI dealer, distributor, agent
//  or employee is authorized to make any modification or addition to this warranty.
//  In no event shall NDI nor any of its employees be liable for any direct, indirect,
//  incidental, special, exemplary, or consequential damages, sundry damages or any
//  damages whatsoever, including, but not limited to, procurement of substitute goods or
//  services, loss of use, data or profits, or business interruption, however caused. In no
//  event shall NDI's liability to the licensee exceed the amount paid by the licensee for the
//  Sample Code or any NDI products that accompany the Sample Code. The said limitations
//  and exclusions of liability shall apply whether or not any such damages are construed as
//  arising from a breach of a representation, warranty, guarantee, covenant, obligation,
//  condition or fundamental term or on any theory of liability, whether in contract, strict
//  liability, or tort (including negligence or otherwise) arising in any way out of the use of
//  the Sample Code even if advised of the possibility of such damage. In no event shall
//  NDI be liable for any claims, losses, damages, judgments, costs, awards, expenses or
//  liabilities of any kind whatsoever arising directly or indirectly from any injury to person
//  or property, arising from the Sample Code or any use thereof
//
//----------------------------------------------------------------------------

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "BufferedReader.h"
#include "CombinedApi.h"
#include "ComConnection.h"
#include "GbfContainer.h"
#include "GbfFrame.h"
#include "SystemCRC.h"
#include "TcpConnection.h"

#include "UdpSocket.h"
#include "FileConnection.h"

#ifdef OPENSSL
#include "DtlsConnection.h"
#include "TlsConnection.h"
#endif

CombinedApi::CombinedApi()
{
    commandChannel_ = NULL;
    streamMap_ = std::map<std::string, Connection*>();
    crcValidator_ = new SystemCRC();
}

CombinedApi::~CombinedApi()
{
    stopStreaming(); // USTREAM and free any open streams before closing the _connection
    delete commandChannel_;
    delete crcValidator_;
}

int CombinedApi::connect(std::string hostname, Protocol protocol, std::string cipher)
{
    std::cout << "Connecting to " << hostname << " ..." << std::endl;

    // Delete any old connection
    if (commandChannel_ != NULL)
    {
        delete commandChannel_;
        commandChannel_ = NULL;
    }

    // Determine if the device uses serial or ethernet communication
    int errorCode = -1;
    if (hostname.substr(0,3).compare("COM") == 0 || hostname.substr(0,4).compare("/dev") == 0)
    {
        // Create a new ComConnection
        commandChannel_ = new ComConnection(hostname);

        // Once the connection is open, the host and device need to agree on a baud rate
        if (commandChannel_->isConnected())
        {
            // A serial break is sent to reset the device to the default 9600 baud rate
            static_cast<ComConnection*>(commandChannel_)->sendSerialBreak();

            // Wait for the system to reply RESET
            errorCode = getErrorCodeFromResponse(readResponse());

            // Print the firmware version for debugging purposes
            std::cout << "API Revision: " << getApiRevision() << std::endl;

            // The host can now request the device go to a much faster baud rate...
            if (errorCode == 0)
            {
                std::cout << "Setting Baud921600 for compatibility. Check your API guide to see if this is optimal for your NDI device." << std::endl;
                errorCode = setCommParams(CommBaudRateEnum::Baud921600);
            }
        }
    }
    else
    {
        // Conditionally compile TlsConnection so users can build the sample without installing OpenSSL, if desired
        if (protocol == Protocol::SecureTCP)
        {
#ifdef OPENSSL
            commandChannel_ = new TlsConnection(hostname.c_str(), "8764", cipher.c_str());
            errorCode = commandChannel_->isConnected() ? 0 : -1;
#else
            // Right click the library solution > Properties > C++ > All Options > Preprocessor Definitions > add "OPENSSL" to the semicolon delimited list
            std::cout << "Rebuild with the preprocessor definition OPENSSL to link installed OpenSSL libraries for TLS support," << std::endl
                << "or re-run the sample and omit the --encrypted option to run the sample without demonstrating TLS support." << std::endl;
            errorCode = -1;
#endif
        }
        else
        {
            commandChannel_ = new TcpConnection(hostname.c_str(), "8765");
            errorCode = commandChannel_->isConnected() ? 0 : -1;
        }
    }

    return errorCode;
}

int CombinedApi::setCommParams(CommBaudRateEnum::value baudRate, int dataBits, int parity, int stopBits, int enableHandshake) const
{
    // Send the COMM command
    std::string command =  std::string("COMM ").append(intToString(baudRate)).append(intToString(dataBits)).append(intToString(parity))
                                               .append(intToString(stopBits)).append(intToString(enableHandshake));
    sendCommand(command);

    // Wait for the system to reply OKAY
    int errorCode = getErrorCodeFromResponse(readResponse());

    // If the device agreed to change the baud rate as desired, set the baud rate on the host to match it
    if (errorCode == 0)
    {
        // std::cout << "Debug: configuring host baud rate..." << std::endl;
        dynamic_cast<ComConnection*>(commandChannel_)->setSerialPortParams(CommBaudRateEnum::toInt(baudRate), (dataBits > 0 ? 7 : 8), parity, stopBits, enableHandshake);
    }
    return errorCode;
}

std::string CombinedApi::getApiRevision() const
{
    // Send the APIREV command
    std::string command =  std::string("APIREV ");
    sendCommand(command);

    // Return the raw string so the client can parse the version
    return readResponse();
}

int CombinedApi::initialize() const
{
    // Send the INIT command
    std::string command =  std::string("INIT ");
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

std::vector<PortHandleInfo> CombinedApi::portHandleSearchRequest(PortHandleSearchRequestOption::value option) const
{
    // Send the PHSR command
    std::stringstream stream;
    stream << "PHSR " << std::setw(2) << std::setfill('0') << (int) option;
    std::string command = stream.str();
    sendCommand(command);

    // If an error occurred, return an empty vector
    std::string response = readResponse();
    int errorCode = getErrorCodeFromResponse(response);
    std::vector<PortHandleInfo> portHandleInfoVector;
    if (errorCode != 0)
    {
        std::cout << response << " - " << errorToString(errorCode);
        return portHandleInfoVector;
    }

    // Return the response as a std::vector of easy to use objects
    int numPortHandles = stringToInt(response.substr(0,2));
    for (int i = 0; i < numPortHandles; i ++)
    {
        portHandleInfoVector.push_back(PortHandleInfo(response.substr(i * 5 + 2, 2), (uint8_t) stringToInt(response.substr(i * 5 + 4, 3))));
    }

    return portHandleInfoVector;
}

int CombinedApi::portHandleFree(std::string portHandle) const
{
    // If the port handle is invalid, print an error message and return
    if (portHandle.size() != 2)
    {
        return -1;
    }

    // Send the PHF command
    std::string command =  std::string("PHF ").append(portHandle);
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::portHandleRequest(std::string hardwareDevice, std::string systemType, std::string toolType, std::string portNumber, std::string dummyTool) const
{
    // Send the PHRQ command
    std::string command =  std::string("PHRQ ").append(hardwareDevice).append(systemType).append(toolType).append(portNumber).append(dummyTool);
    sendCommand(command);

    // Return the requested port handle or an error code
    std::string response = readResponse();
    int errorCode = getErrorCodeFromResponse(response);
    if (errorCode == 0)
    {
        return stringToInt(response);
    }
    else
    {
        return errorCode;
    }
}

int CombinedApi::portHandleInitialize(std::string portHandle) const
{
    // If the port handle is invalid, print an error message and return
    if (portHandle.size() != 2)
    {
        return -1;
    }

    // Send the PINIT command
    std::string command =  std::string("PINIT ").append(portHandle);
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::portHandleEnable(std::string portHandle, ToolTrackingPriority::value priority) const
{
    // If the port handle is invalid, print an error message and return
    if (portHandle.size() != 2)
    {
        return -1;
    }

    // Send the PENA command
    std::string command =  std::string("PENA ").append(portHandle);
    command += (char)priority;
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

PortHandleInfo CombinedApi::portHandleInfo(std::string portHandle) const
{
    // If the port handle is invalid, print an error message and return
    if (portHandle.size() != 2)
    {
        std::cout << "Invalid port handle: " << portHandle << std::endl;
        return PortHandleInfo(portHandle);
    }

    // Send the PHINF command
    std::string command =  std::string("PHINF ").append(portHandle);
    sendCommand(command);

    // If there is no tool loaded, return an empty PortHandleInfo
    std::string response = readResponse();
    int errorCode = getErrorCodeFromResponse(response);
    if (errorCode != 0)
    {
        std::cout << response << " - " << errorToString(errorCode);
        return PortHandleInfo(portHandle);
    }
    else if (response.substr(0,10).compare("UNOCCUPIED") == 0)
    {
        std::cout << "No tool loaded at port: " << portHandle << std::endl;
        return PortHandleInfo(portHandle);
    }

    // Parse the information from the response
    std::string toolType = response.substr(0,8);
    std::string toolId = response.substr(8,12);
    toolId.erase(toolId.find_last_not_of(" \n\r\t") + 1); // .trim() whitespace
    std::string revision = response.substr(20,3);
    std::string serialNumber = response.substr(23,8);
    uint8_t status = (uint8_t) stringToInt(response.substr(31,2));
    return PortHandleInfo(portHandle, toolType, toolId, revision, serialNumber, status);
}

int CombinedApi::loadPassiveDummyTool() const
{
    return portHandleRequest("********", "*", "1", "00", "01");
}

int CombinedApi::loadActiveWirelessDummyTool() const
{
    return portHandleRequest("********", "*", "1", "00", "02");
}

int CombinedApi::loadActiveDummyTool() const
{
    return portHandleRequest("********", "*", "0", "00", "01");
}

void CombinedApi::loadSromToPort(std::string romFilePath, int portHandle) const
{
    // If the port handle is invalid, print an error message and return
    if (portHandle < 0)
    {
        std::cout << "Invalid port handle: " << portHandle << std::endl;
        return;
    }

    // If the .rom file cannot be opened, print an error message and return
    std::ifstream inputFileStream(romFilePath.c_str(), std::ios_base::binary);
    if (!inputFileStream.is_open())
    {
        std::cout << "Cannot open file: " + romFilePath << std::endl;
        return;
    }

    // Read the entire file and convert it to ASCII hex characters
    std::stringstream romStream;
    romStream << std::setfill('0') << std::hex;
    while (!inputFileStream.eof())
    {
        romStream << std::setw(2)  << inputFileStream.get();
    }
    inputFileStream.close();

    // Tool data is sent in chunks of 128 hex characters (64-bytes).
    // It must be an integer number of chunks, padded with zeroes at the end.
    const int messageSizeChars = 128;
    const int messageSizeBytes = 64;
    std::string toolDefinition = romStream.str();
    int remainder = toolDefinition.size() % messageSizeChars;
    toolDefinition.append((messageSizeChars - remainder), '0');
    const int totalIterations =  (int)toolDefinition.size() / messageSizeChars;
    std::string command = "";
    int errorCode = 0;
    std::stringstream startAddressStream;
    startAddressStream << std::setfill('0') << std::hex;
    for (int i = 0; i < totalIterations; i++)
    {
        // Pass the startAddress as a fixed width four characters of hex padded with zeroes
        startAddressStream << std::setw(4) <<  i * messageSizeBytes;

        // Send the PVWR command
        command =  std::string("PVWR ").append(intToHexString(portHandle, 2));
        command += startAddressStream.str();
        command += toolDefinition.substr(i * messageSizeChars, messageSizeChars);
        sendCommand(command);

        // If we run into an error, print something before exiting
        errorCode = getErrorCodeFromResponse(readResponse());
        if (errorCode != 0)
        {
            std::cout << "PVWR returned error: " << errorToString(errorCode) << std::endl;
            return;
        }

        // Reset the stringstream used to print fixed width address sizes
        startAddressStream.str("");
    }
}

int CombinedApi::startTracking() const
{
    // Send the TSTART command
    std::string command =  std::string("TSTART ");
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::stopTracking() const
{
    // Send the TSTOP command
    std::string command =  std::string("TSTOP ");
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

int CombinedApi::startStreaming(const std::string cmd, const std::string streamId, Protocol protocol, std::string cipher)
{
    // Append parameters to the command. --protocol=TCP is the default, but it must be specified if UDP is desired
    std::string command = std::string("STREAM").append(" --cmd=\"").append(cmd).append("\"");

    // The command string is used as the streamId if the streamId is not provided
    if (!streamId.empty())
    {
        command.append(" --id=").append(streamId);
    }

    // The command channel already knows the hostname we want to talk to
    std::string hostname = commandChannel_->connectionName();
    hostname = hostname.substr(0, hostname.find(':'));

    // Open the streaming connection
    Connection *streamConnection;
    std::string response = "";
    if (protocol == Protocol::TCP)
    {
        streamConnection = new TcpConnection(hostname.c_str(), "8765");
    }
    else if (protocol == Protocol::UDP)
    {
        // Choose a local port to ask the system to stream to, like 54321 for example
        command.append( " --protocol=UDP --port=54321" );
        streamConnection = new UdpSocket(hostname.c_str(), "54321" );
    }
    else // SecureTCP or SecureUDP
    {
// Conditionally compile DtlsConnection so users can build the sample without installing OpenSSL, if desired
#ifdef OPENSSL
        if (protocol == Protocol::SecureTCP)
        {
            streamConnection = new TlsConnection(hostname.c_str(), "8764", cipher.c_str());
        }
        else if (protocol == Protocol::SecureUDP)
        {
            streamConnection = new DtlsConnection( hostname.c_str(), "8764", cipher.c_str() );
            if ( !streamConnection->isConnected() )
            {
                // Try again
                std::cerr << "FAILED to connect and stream using DTLS. Trying again..." << std::endl;
                streamConnection = new DtlsConnection( hostname.c_str(), "8764", cipher.c_str() );
                if ( !streamConnection->isConnected() )
                {
                    // Return error code
                    std::cerr << "FAILED to connect and stream using DTLS." << std::endl;
                    return 1;
                }
            }
        }
        else
        {
            std::cerr << "Unrecognized protocol type" << std::endl;
            return -1;
        }
        response = streamConnection->isConnected() ? "" : "WARNING01"; // possible hardware fault (no device)
#else
    // Right click the library solution > Properties > C++ > All Options > Preprocessor Definitions > add "OPENSSL" to the semicolon delimited list
    std::cout << "Rebuild with the preprocessor definition OPENSSL to link installed OpenSSL libraries for TLS support," << std::endl
        << "or re-run the sample and omit the --encrypted option to run the sample without demonstrating TLS support." << std::endl;
    return getErrorCodeFromResponse("ERROR01");// invalid command (library is inaccessible)
#endif
    }

    // Keep track of open streams
    streamMap_.insert(std::make_pair(streamId, streamConnection));

    // Send the STREAM command to the connection that was just opened
    if ( protocol == Protocol::TCP || protocol == Protocol::SecureTCP || protocol == Protocol::SecureUDP )
    {
        sendCommand( command, streamConnection );
        response = readStream( streamId );
    }
    else
    {
        // Send the command on the original TCP/TLS connection and listen on the UDP connection
        sendCommand( command );
        response = readResponse();
    }

    // Check that the stream command worked
    if (response.compare("OKAY") != 0)
    {
        std::cerr << "Command failed: " << command << " on connection: " << streamConnection->connectionName() << std::endl;
    }

    // Return the error code from the STREAM command
    return getErrorCodeFromResponse(response);
}

int CombinedApi::stopStreaming(const std::string streamId, Protocol protocol )
{
    std::map<std::string, Connection*>::const_iterator it;
    std::string command = std::string("USTREAM");
    if (!streamId.empty())
    {
        // Try to find the given key in the map
        it = streamMap_.find(streamId);
        if (it == streamMap_.end()) return -1; // key not found

        // Send USTREAM comamnd
        command.append(" --id=").append(streamId);
        std::string reply = "";
        if ( protocol == Protocol::UDP )
        {
            // send on the main TCP channel
            sendCommand( command );
            reply = readResponse();
        }
        else
        {
            // Send on the channel used for streaming
            sendCommand( command, it->second );
            reply = readResponse( it->second );
        }

        // Free memory allocated to the Connection, and remove it from the map
        delete it->second;
        streamMap_.erase(it);

        // Return the error code for the USTREAM
        return getErrorCodeFromResponse(reply);
    }
    else
    {
        // Blank streamId means close all open streams
        bool errorDetected = false;
        std::string response = "";
        for (it = streamMap_.begin(); it != streamMap_.end(); it++ )
        {
            // USTREAM each element in the map
            command = std::string("USTREAM --id=").append(it->first);
            sendCommand(command);
            response = readResponse();

            // If any USTREAM has gone wrong, flag it
            errorDetected |= getErrorCodeFromResponse(response) != 0;
            delete it->second; // free memory allocated to the Connection
        }

        // Remove all key-value pairs from the map
        streamMap_.clear();

        // Return success(0) or error(-1) if any command had a problem
        return errorDetected ? -1 : 0;
    }
}

std::string CombinedApi::readStream(std::string streamId) const
{
    // Lookup the stream
    if ( streamMap_.find( streamId ) == streamMap_.end() )
    {
        return "ERROR: Stream \"" + streamId + "\" not found.";
    }

    Connection* streamingConnection = streamMap_.at(streamId);

    // Copy the response into the string
    std::string response = "";
    if (streamingConnection->protocol == StreamingProtocol::UDP)
    {
        // Read a packet
        char packet[2048] = { '\0' }; // choose a reasonable packet size (eg. 2048 bytes)
        int bytes_read = streamingConnection->read(packet, sizeof(packet));
        response = std::string(packet, bytes_read);
    }
    else
    {
        // Read from the device until we encounter a terminating carriage return (CR)
        char lastChar = '\0';
        while (lastChar != CR)
        {
            int bytesRead = streamingConnection->read(&lastChar, 1);
            if ( bytesRead <= 0 )
            {
                // Timeout!
                std::cerr << "ERROR: Streaming timeout." << std::endl;
                return "ERROR: Streaming timeout.";
            }
            response += lastChar;
        }
    }

    // Trim trailing CR and verify the CRC16
    response.erase(response.length() - 1); // strip CR (1 char)
    unsigned int replyCRC16 = (unsigned int)stringToInt(response.substr(response.length() - 4, 4));
    response.erase(response.length() - 4, 4); // strip CRC16 (4 chars)
    if (crcValidator_->calculateCRC16(response.c_str(), (int)response.length()) != replyCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
    }

    // If the response is not a simple "OKAY" to the "STREAM" command
    if( response.length() > 4 || response.substr( 0, 4 ).compare( "OKAY" ) > 0 )
    {
        // strip the header
        response.erase( 0, streamId.length() + 6 );
    }

    // send response to terminal
    std::cout << "<<" << response << std::endl;
    return response;
}

std::string CombinedApi::getTrackingDataTX(const uint16_t options) const
{
    // Send the TX command
    std::string command =  std::string("TX ").append(intToHexString(options, 4));
    sendCommand(command);
    return readResponse();
}

std::vector<ToolData> CombinedApi::getTrackingDataBX(const uint16_t options) const
{
    // Send the BX command
    std::string command =  std::string("BX ").append(intToHexString(options, 4));
    sendCommand(command);

    // Open a buffered reader on the connection to easily parse the binary reply
    BufferedReader reader(commandChannel_);
    reader.readBytes(6);
    uint16_t startSequence = reader.get_uint16();
    uint16_t replyLengthBytes = reader.get_uint16();

    // Verify the CRC16 of the header
    unsigned int headerCRC16 = (unsigned int) reader.get_uint16();
    unsigned int calculatedCRC16 = crcValidator_->calculateCRC16(reader.getData(0, 4).c_str(), 4);
    if (calculatedCRC16 != headerCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
        return std::vector<ToolData>();
    }

    // In the case of an unexpected binary header, return an empty vector
    if (startSequence != START_SEQUENCE)
    {
        std::cout << "Unrecognized start sequence: " << startSequence << std::endl;
        return std::vector<ToolData>();
    }

    // Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
    reader.readBytes(replyLengthBytes + 2);

    // Verify the CRC16 of the data
    reader.skipBytes(replyLengthBytes);
    unsigned int dataCRC16 = reader.get_uint16();
    if (crcValidator_->calculateCRC16(reader.getData(6, replyLengthBytes).c_str(), replyLengthBytes) != dataCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
        return std::vector<ToolData>();
    }
    reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

    // Debugging: print the raw binary and/or interpreted strings
    /*std::cout << reader.toString() << std::endl;
    std::cout << "startSequence=" << intToHexString(startSequence, 4) << std::endl
           << "replyLengthBytes=" << intToHexString(replyLengthBytes, 4) << std::endl
           << "headerCRC16=" << intToHexString(headerCRC16, 4) << std::endl;*/

    // TODO: support all BX options. Just return if there are unexpected options, we will be binary misaligned anyway.
    if ((options & ~(TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms)) != 0x0000)
    {
        std::cout << "Reply parsing has not implemented options: " << intToHexString(options, 4) << std::endl;
        return std::vector<ToolData>();
    }

    std::vector<ToolData> toolDataVector;
    uint8_t numHandles = reader.get_byte();
    for (uint8_t i = 0; i < numHandles; i++)
    {
        // Create a new ToolData
        toolDataVector.push_back(ToolData());

        // From each two byte handle, extract the handle index and status
        toolDataVector.back().transform.toolHandle = (uint16_t) reader.get_byte();
        uint8_t handleStatus = reader.get_byte();

        // Parse BX 0001 - See API guide for protocol details
        if (options & TrackingReplyOption::TransformData)
        {
            // The transform is not transmitted at all if it is missing
            switch (handleStatus)
            {
                case 0x01: // Valid
                    toolDataVector.back().transform.status = TransformStatus::Enabled;
                    toolDataVector.back().transform.q0 = reader.get_double();
                    toolDataVector.back().transform.qx = reader.get_double();
                    toolDataVector.back().transform.qy = reader.get_double();
                    toolDataVector.back().transform.qz = reader.get_double();
                    toolDataVector.back().transform.tx = reader.get_double();
                    toolDataVector.back().transform.ty = reader.get_double();
                    toolDataVector.back().transform.tz = reader.get_double();
                    toolDataVector.back().transform.error = reader.get_double();
                break;
                case 0x04: // Disabled
                    // Disabled markers have no transform, status, or frame number
                    toolDataVector.pop_back(); // don't return a ToolData for it, there's nothing there
                    continue;
                default:
                    // case 0x02: Missing or anything unexpected
                    // do nothing --> blank Transform object is already initialized as missing
                break;
            };

            // Regardless of transform status, there is info about the port and frame
            toolDataVector.back().portStatus = reader.get_uint32() & 0x0000FFFF;
            toolDataVector.back().frameNumber = reader.get_uint32();
        }
    }

    // Add the systemStatus to each ToolData
    uint16_t systemStatus = reader.get_uint16();
    for (int t = 0; t < toolDataVector.size(); t++)
    {
        toolDataVector[t].systemStatus = systemStatus;
    }

    // Return the tool data
    return toolDataVector;
}

std::vector<ToolData> CombinedApi::getTrackingDataBX2(std::string options) const
{
    // Send the BX2 command
    std::string command =  std::string("BX2 ").append(options);
    sendCommand(command);

    // Open a buffered reader on the connection to easily parse the binary reply
    BufferedReader reader(commandChannel_);

    // The BX2 reply begins with a 6 byte header:
    // (2-bytes) StartSequence: indicates how to parse the reply. A5C4 (normal)
    // (2-bytes) ReplyLength: length of the reply in bytes
    // (2-bytes) CRC16
    reader.readBytes(6);
    uint16_t startSequence = reader.get_uint16();
    uint16_t replyLengthBytes = reader.get_uint16();

    // Verify the CRC16 of the header
    unsigned int headerCRC16 = (unsigned int) reader.get_uint16();
    unsigned int calculatedCRC16 = crcValidator_->calculateCRC16(reader.getData(0, 4).c_str(), 4);
    if (calculatedCRC16 != headerCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
        return std::vector<ToolData>();
    }

    // TODO: handle all BX2 reply types?
    if (startSequence != START_SEQUENCE)
    {
        // Return an empty vector if the binary response cannot be interpreted
        std::cout << "Unrecognized BX2 reply header: " << std::setw(4) << startSequence << " - Not implemented yet!" << std::endl;
        return std::vector<ToolData>();
    }

    // Get all of the data once we know how many bytes to read: replyLengthBytes + 2 bytes for trailing CRC16
    reader.readBytes(replyLengthBytes + 2);

    // Verify the CRC16 of the data
    reader.skipBytes(replyLengthBytes);
    unsigned int dataCRC16 = reader.get_uint16();
    if (crcValidator_->calculateCRC16(reader.getData(6, replyLengthBytes).c_str(), replyLengthBytes) != dataCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
        return std::vector<ToolData>();
    }
    reader.skipBytes(-replyLengthBytes -2); // move the BufferedReader's pointer back so we can parse the data

    // Parse the binary into meaningful objects
    GbfContainer container(reader);

    // Debugging: print the raw binary and/or interpreted strings
    /*std::cout << reader.toString() << std::endl;
    std::cout << "-----GbfHeader" << std::endl
              << "startSequence=" << std::setw(4) << static_cast<unsigned>(startSequence) << std::endl
              << "replyLengthBytes=" << std::setw(4) << static_cast<unsigned>(replyLengthBytes) << std::endl
              << "headerCRC16=" << std::setw(4) << static_cast<unsigned>(headerCRC) << std::endl
              << container.toString()
              << "dataCRC16=" << std::setw(4) << static_cast<unsigned>(reader.get_uint16()) << std::endl << std::endl;*/

    // Search the root GbfContainer to find the frame component
    std::vector<ToolData> retVal;
    for (int i = 0; i < container.components.size(); i++)
    {
        if (container.components[i]->componentType == GbfComponentType::Frame)
        {
            // Every GBF frame has GbfFrameDataItems for each type of tool: Passive, ActiveWireless, Active
            GbfFrame* frame = static_cast<GbfFrame*>(container.components[i]);
            retVal = frame->getToolData();
      break;
        }
    }

    // If we didn't find any, then return an empty vector
    return retVal;
}

std::string CombinedApi::intToString(int input, int width) const
{
    std::stringstream convert;
    convert << std::dec << std::setfill('0');
    convert << std::setw(width) << input;
    return convert.str();
}

std::string CombinedApi::intToHexString(int input, int width) const
{
    std::stringstream convert;
    convert << std::hex  << std::setfill('0');
    convert << std::setw(width) << input;
    return convert.str();
}

int CombinedApi::stringToInt(std::string input) const
{
    int retVal = 0;
    std::stringstream convert(input);
    convert << std::hex;
    convert >> retVal;
    return retVal;
}

char *CombinedApi::getConnectionName()
{
    if (!commandChannel_)
    {
        return NULL;
    }
    return commandChannel_->connectionName();
}

int CombinedApi::getErrorCodeFromResponse(std::string response) const
{
    // Parse the error code from the response string and return it
    int errorCode = 0;
    if (response.substr(0,5).compare("ERROR") == 0)
    {
        errorCode = stringToInt(response.substr(5,2));
    }
    else if (response.substr(0,7).compare("WARNING") == 0)
    {
        errorCode = stringToInt(response.substr(7,2)) + WARNING_CODE_OFFSET;
    }

    // Use negative error codes to distinguish between port handles an errors.
    return errorCode * -1;
}

std::string CombinedApi::readResponse(Connection* connection) const
{
    // If the connection doesn't exist, just fail
    if (connection == nullptr)
    {
        if (commandChannel_ == nullptr)
        {
            std::cout << "Cannot read response. Connection is null!" << std::endl;
            return "";
        }
        else
        {
            // Use the command channel by default
            connection = commandChannel_;
        }
    }

    // Declare an empty string to hold the response
    std::string response = std::string("");
    char lastChar = '\0';

    // Read from the device until we encounter a terminating carriage return (CR)
    while (lastChar != CR)
    {
        int numBytes = connection->read(&lastChar, 1);
        if ( numBytes <= 0 )
        {
            // Timeout!
            std::cerr << "ERROR: Reply timeout." << std::endl;
            return "ERROR: Reply timeout.";
        }
        response += lastChar;
    }

    // Trim trailing CR and verify the CRC16
    response.erase(response.length() - 1); // strip CR (1 char)
    unsigned int replyCRC16 = (unsigned int) stringToInt(response.substr(response.length() - 4 , 4));
    response.erase(response.length() - 4, 4); // strip CRC16 (4 chars)
    if (crcValidator_->calculateCRC16(response.c_str(), (int) response.length()) != replyCRC16)
    {
        std::cout << "CRC16 failed!" << std::endl;
    }
    
    // send response to terminal
    std::cout << "<<" << response << std::endl;

    // Return whatever string the device responded with
    return response;
}

int CombinedApi::sendCommand(std::string command, Connection* connection) const
{
    std::cout << "CombinedApi::sendCommand" << std::endl;

    // If the connection doesn't exist, just fail
    if (connection == nullptr)
    {
        if (commandChannel_ == nullptr)
        {
            std::cout << "Cannot send command: " << command << "- Connection is null!" << std::endl;
            return -1;
        }
        else
        {
            // Use the command channel by default
            connection = commandChannel_;
        }
    }

    // Display an error message if there is no open socket
    if (!connection->isConnected())
    {
        std::cout << "Cannot send command: " << command << "- No open socket!" << std::endl;
        return -1;
    }

    // Display the command that we're sending (except for BX, slows us down for real use)
    if (command.find("BX") == std::string::npos)
    {
        std::cout << "Sending command: " << command << " ..." << std::endl;
    }

    // Add CR character to command and write the command to the socket
    command += CR;
    return connection->write(command.c_str(), (int)command.length());
}

std::string CombinedApi::errorToString(int errorCode)
{
    errorCode *= -1; // restore the errorCode to a positive value
    if (errorCode > WARNING_CODE_OFFSET)
    {
        return getWarningString(errorCode - WARNING_CODE_OFFSET);
    }
    else
    {
        return getErrorString(errorCode);
    }
}

std::string CombinedApi::getWarningString(int warnCode)
{
    if (warnCode < 0 || warnCode >= sizeof(warningStrings))
    {
        return "Warning code not found.";
    }
    else
    {
        return warningStrings[warnCode];
    }
}

std::string CombinedApi::getErrorString(int errorCode)
{
    if (errorCode < 0 || errorCode >= sizeof(errorStrings))
    {
        return "Error code not found.";
    }
    else
    {
        return errorStrings[errorCode];
    }
}

std::string CombinedApi::getUserParameter(std::string paramName) const
{
    // Send the GET request
    std::string command = std::string("GET ").append(paramName);
    sendCommand(command);

    // Return whatever the device responded with
    return readResponse();
}

int CombinedApi::setUserParameter(std::string paramName, std::string value) const
{
    // Send the SET request
    std::string command = std::string("SET ").append(paramName).append("=").append(value);
    sendCommand(command);
    return getErrorCodeFromResponse(readResponse());
}

int CommBaudRateEnum::toInt(CommBaudRateEnum::value baudEnumValue)
{
    switch (baudEnumValue)
    {
        case Baud1228739:
            return 1228739;
        case Baud921600:
            return 921600;
        case Baud115200:
            return 115200;
        case Baud57600:
            return 57600;
        case Baud38400:
            return 38400;
        case Baud19200:
            return 19200;
        case Baud14400:
            return 14400;
        case Baud9600: // fall through
        default:
            return 9600;
    };
}

int CombinedApi::convertGbfFileToText( char* inputGBFFile, char* outputTxtFile )
{
    // Open a "connection" to the input file, as if it were a communications channel
    FileConnection fc;
    fc.connect( inputGBFFile );

    // Create a buffered reader to read through the input file
    BufferedReader reader( &fc );

    // Check the file length
    if( fc.getBytesRemaining() < 12 )
    {
        std::cerr << "File does not contain GBF header." << std::endl;
        return -1;
    }

    // "GBF\0" file header
    reader.readBytes( sizeof( uint32_t ) );
    int fileTypeInt = reader.get_uint32();
    int supportedFileType = 0x00464247; // GBF\0 little-endian
    if( fileTypeInt != supportedFileType )
    {
        std::cerr << "Incompatible file type." << std::endl;
        return -1;
    }

    // File format version
    reader.readBytes( sizeof( uint32_t ) );
    int fileVer = reader.get_uint32();
    if ( fileVer < 1 || fileVer > 2 )
    {
        std::cerr << "Incompatible GBF file format version: " << fileVer << std::endl;
        return -1;
    }

    // Number of samples in the file
    reader.readBytes( sizeof( uint32_t ) );
    int numSamples = reader.get_uint32();

    // Open the output text file
    std::ofstream output;
    output.open( outputTxtFile );

    // If newer than version 1
    if ( fileVer > 1 )
    {
        // Read the tagged values packet size
        reader.readBytes( sizeof( uint32_t ) );
        int sizeTaggedValues = reader.get_uint32();

        // Read the bytes into a string
        reader.readBytes( sizeTaggedValues );
        std::string taggedValuesStr = reader.get_string( sizeTaggedValues );
        output << taggedValuesStr << std::endl;
    }

    int nSamplesRead = 0;

    try
    {
        // If there is at least one GBF container left in the file
        while ( fc.getBytesRemaining() > 10 && nSamplesRead < numSamples )
        {
            // Read the version, number of components, component type and packet size
            reader.readBytes( sizeof( uint16_t ) );
            // If the container follows GBF version 1 schema
            short gbfVer = reader.get_uint16();
            if ( gbfVer == 1 )
            {
                nSamplesRead++;

                // Read more of the header to get to the size of the container
                reader.readBytes( sizeof( uint16_t ) );
                short nComponents = reader.get_uint16();
                if ( nComponents <= 0 )
                {
                    continue;
                }
                // Read the component type and size from the file
                reader.readBytes( sizeof( uint16_t ) + sizeof( uint32_t ) );
                short compType = reader.get_uint16();
                int compSize = reader.get_uint32();

                // Read the rest of the GbfContainer into the buffer
                reader.readBytes( compSize - sizeof( uint16_t ) - sizeof( uint32_t ) );

                // Skip back in the buffer so the GbfContainer parser will read the Gbf version and the rest of the header
                reader.skipBytes( 0 -( sizeof( uint32_t ) * 2 ) - sizeof( uint16_t) );

                // Parse the GBF Container from the reader's buffer
                GbfContainer container( reader );

                // Print the container to the output file
                output << container.toString() << std::endl;
            }
            else
            {
                std::cerr << "GBF Version " << gbfVer << " not supported!" << std::endl;
                fc.disconnect();
                output.close();
                return -1;
            }
        }
    }
    catch ( const std::exception& ex )
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        output.close();
    }
    fc.disconnect();
    output.close();
    return 0;
}