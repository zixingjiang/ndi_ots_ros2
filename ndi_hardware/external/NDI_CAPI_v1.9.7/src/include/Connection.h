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
#ifndef CONNECTION_HPP
#define CONNECTION_HPP


#ifdef _WIN32
// Include the DLL for socket programming
// Confusingly, the 64-bit library has 32 in its name as well...
// See: http://stackoverflow.com/questions/5507607/winsock-and-x64-target
#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h> // for Windows Socket API (WSA)
#include <ws2tcpip.h> // for getaddrinfo() etc...

#else
// Mac and Linux use a different API for sockets
#define SOCKET int // Windows SOCKET is unsigned
#define INVALID_SOCKET -1 // Windows uses unsigned, so this is "all bits set"
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdint.h> // for uint8_t etc...

/**
 * C++ doesn't actually define a byte as 8-bits, but rather a number of bits that can
 * fit the entire character set. Here we define a byte as an 8 bit integer.
 */
typedef uint8_t byte_t;

//! Describes which protocol the socket uses
enum class StreamingProtocol { TCP, UDP, SecureTCP, SecureUDP };

/**
 * @brief A common interface for communication classes.
 */
class Connection
{
public:

    //! Virtual destructor ensures that calling derived destructors behave properly
    virtual ~Connection() {};

    /** @brief Returns true if the connection succeeded */
    virtual bool isConnected() const = 0;

    /** @brief Closes the connection */
    virtual void disconnect() = 0;

    /**
     * @brief Returns true if the socket is a valid file descriptor
     * @param socket The socket to evaluate
     */
    static bool socketIsValid(SOCKET socket);

    /**
     * @brief Reads 'length' chars  from the socket into 'buffer'
     * @param buffer The buffer to read into.
     * @param length The number of chars to read.
     */
    virtual int read(char* buffer, int length) const = 0;

    /**
     * @brief Reads 'length' bytes from the socket into 'buffer'
     * @param buffer The buffer to read into.
     * @param length The number of bytes to read.
     */
    virtual int read(byte_t* buffer, int length) const = 0;

    /**
     * @brief Writes 'length' chars from 'buffer' to the socket
     * @param buffer The buffer to write from.
     * @param length The number of chars to write.
     */
    virtual int write(const char* buffer, int length) const = 0;

    /**
     * @brief Writes 'length' bytes from 'buffer' to the socket
     * @param buffer The buffer to write from.
     * @param length The number of bytes to write.
     */
    virtual int write(byte_t* buffer, int length) const = 0;

    /** @brief Gets the "hostname:port" or "IpAddress:port" string */
    virtual char *connectionName() = 0;

    //! Specifies the protocol to use
    StreamingProtocol protocol = StreamingProtocol::TCP;
private:
};

#endif // CONNECTION_HPP
