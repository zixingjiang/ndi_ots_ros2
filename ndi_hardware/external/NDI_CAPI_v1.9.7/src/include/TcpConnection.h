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
#ifndef TCP_CONNECTION_HPP
#define TCP_CONNECTION_HPP

#include "Connection.h"

/**
 * @brief A cross platform socket implementation.
 */
class TcpConnection : public Connection
{
public:

    /** Constructs an empty socket object. */
    TcpConnection();

    /**
     * @brief Constructs a socket object and connects immediately.
     * @param hostname The hostname or IP address of the measurement system.
     * @param port The port to connect on. Port 8765 is default for Vega systems.
     */
    TcpConnection(const char* hostname, const char* port = "8765");

    /**
     * @brief Closes the socket connection and frees memory.
     */
    virtual ~TcpConnection();

    /**
     * @brief Closes any existing connection, and connects to the new device.
     * @param hostname The hostname or IP address of the device.
     * @param port The port number to connect on.
     */
    bool connect(const char* hostname, const char* port);

    //! @copydoc Connection::disconnect()
    void disconnect();

    //! @copydoc Connection::isConnected()
    bool isConnected() const;

    //! @copydoc Connection::read(byte_t*, int) const
    int read(byte_t* buffer, int length) const;

    //! @copydoc Connection::read(char*, int) const
    int read(char* buffer, int length) const;

    //! @copydoc Connection::write(byte_t*, int) const
    int write(byte_t* buffer, int length) const;

    //! @copydoc Connection::write(const char*, int) const
    int write(const char* buffer, int length) const;

    //! @copydoc Connection::connectionName()
    char* connectionName();

    /**
     * @brief Listens for incoming TCP stream connections
     * @param port The port to connect on. Port 8765 is default for Vega systems.
     */
    void waitForStream(const std::string port);

private:

    //! Setup method common to all constructors.
    void init();

    //! True if the socket is ready to send/recv
    bool isConnected_ = false;

    //! The IPv4 address of the host
    char ip4_[INET_ADDRSTRLEN] = { '\0' };

    //! The socket file descriptor
    SOCKET ndiSocket_ = 0;

    #ifdef _WIN32 // Windows socket implementation
    WSADATA   wsaData_;
    #endif
};

#endif // TCP_CONNECTION_HPP
