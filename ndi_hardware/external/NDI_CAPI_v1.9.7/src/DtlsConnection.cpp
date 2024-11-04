//----------------------------------------------------------------------------
//
//  Copyright (C) 2021, Northern Digital Inc. All rights reserved.
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
#ifdef OPENSSL

#include <cstring> // for std::cerr
#include <errno.h> // for errno on some linux machines
#include <iostream>
#include <string.h> // for memcpy

#include "DtlsConnection.h"

const SSL_METHOD* DtlsConnection::negotiateProtocol()
{
    return DTLS_method();
}

BIO* DtlsConnection::createBIO()
{
    // Setup the datagram socket first, then wrap it with the BIO...
    fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // A minimum protocol version can be used to forbid communication using old insecure encryption algorithms.
    // This limit is only useful if the failure handling doesn't simply transmit data unencrypted!
    SSL_CTX_set_min_proto_version(ssl_ctx_, DTLS1_2_VERSION);

    // Create the BIO using UDP transport
    return BIO_new_dgram(fd_, BIO_CLOSE);
}

bool DtlsConnection::doConnect()
    {
    union
    {
        struct sockaddr_in s4;
        struct sockaddr_storage ss;
    } client_addr;
    memset( &client_addr, 0, sizeof(struct sockaddr_storage) );

    // Using INADDR_ANY binds to 0.0.0.0:port, meaning the port is bound on ALL interface addresses.
    // Computers often have multiple network interfaces (eg. Ethernet ports, wifi, etc...).
    // If we knew the network interface the Vega was connected on, then we could set this explicitly.
    // bind() requires a unique combination of InterfaceIP:Port, so two different sockets can use the same
    // port if they are on different interfaces.
    std::string hostname = connectionName_.substr(0, connectionName_.find(':'));
    std::string port = connectionName_.substr(connectionName_.find(':') + 1);
    client_addr.s4.sin_addr.s_addr = INADDR_ANY;
    client_addr.s4.sin_port = htons(atoi(port.c_str()));

    // bind() sets the data source address and port
    isConnected_ = ::bind(fd_, (const sockaddr *)&client_addr, sizeof(struct sockaddr_in)) >= 0;

    // Ideally the BIO would provide the IP address translation, but it's null because it isn't connected yet!
    // char* serverIP = connectionIP();

    // Lookup the server IPv4 address from its hostname
    // Note: getaddrinfo returns a list of addresses
    addrinfo addressInfo, *pai, *aiPointer;
    memset(&addressInfo, 0, sizeof(addressInfo));
    addressInfo.ai_family = AF_INET;
    addressInfo.ai_socktype = SOCK_DGRAM;
    addressInfo.ai_protocol = IPPROTO_UDP;

    // Lookup the server IP address using the known hostname and port
    int addrinforesult = getaddrinfo(hostname.c_str(), port.c_str(), &addressInfo, &aiPointer);
    if (addrinforesult != 0)
    {
        std::cerr << "getaddrinfo Error code " << addrinforesult << " (" << gai_strerror(addrinforesult) << ")" << std::endl;
        return false;
    }

    // Technically a whole list of possible interfaces is returned, but we asked for IPv4 (AF_INET) above, so this should be a list with 1 entry.
    // for (pai = aiPointer; pai != NULL; pai = pai->ai_next)

    // Print the IPv4 string for debugging/informational purposes
    char ipv4_address[INET_ADDRSTRLEN] = { '\0' };
    inet_ntop(AF_INET, &((const sockaddr_in*)aiPointer->ai_addr)->sin_addr, ipv4_address, INET_ADDRSTRLEN);

    // connect() sets the destination address which isn't normally required for UDP, but DTLS requires a bidirectional connection!
    int errorCode = ::connect(fd_, aiPointer->ai_addr, (socklen_t) aiPointer->ai_addrlen);
    if (errorCode != 0)
    {
        std::cerr << "connect() failed for DTLS on: " << connectionName_.c_str() << " (" << ipv4_address << ")" << std::endl;
        return false;
    }

    // Tell the BIO the socket is connected
    BIO_ctrl(ssl_bio_, BIO_CTRL_DGRAM_SET_CONNECTED, 0, (sockaddr_storage*)aiPointer->ai_addr);

    // Setup the SSL connection object
    ssl_ = SSL_new(ssl_ctx_);
    if (ssl_ == nullptr)
    {
        std::cerr << "SSL connection object is NULL" << std::endl;
        return false;
    }

    // Set the read & write BIOs on the SSL connection object
    SSL_set_bio(ssl_, ssl_bio_, ssl_bio_);

    // Set connection receive timeout to prevent infinite loop if no data is received
    struct timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;
    BIO_ctrl( ssl_bio_, BIO_CTRL_DGRAM_SET_RECV_TIMEOUT, 0, &timeout );

    // Attempt to connect
    if ( SSL_connect( ssl_ ) < 0 )
    {
        std::cerr << "SSL_connect() failed for DTLS on: " << connectionName_.c_str() << std::endl;
        // Note: if this new connection is attempted within the system's "Param.Connect.Idle Timeout" value,
        // the new connection will fail, because the previous DTLS connection doesn't know that the connection is gone
        // (due to it being a _mostly_ one-way protocol)
        // To deal with this, you can do one of the following:
        // 1. Send RESET 1 to the system to end all connections
        // 2. Shorten the Param.Connect.Idle Timeout (default is 300s) so that you can re-connect sooner
        // 3. Wait the full timeout before attempting a new connection
        return false;
    }

    // Secure connection established!
    return true;
}

DtlsConnection::DtlsConnection(const char* hostname, const char* port, const char* cipher)
{
    protocol = StreamingProtocol::SecureUDP;
    init();
    connect(hostname, port, cipher, SecureProtocol::DTLS);
}

DtlsConnection::~DtlsConnection()
{
    disconnect();
}

#endif // #ifdef OPENSSL
