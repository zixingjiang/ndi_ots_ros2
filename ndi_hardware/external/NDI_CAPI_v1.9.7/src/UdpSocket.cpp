//----------------------------------------------------------------------------
//
//  Copyright (C) 2020, Northern Digital Inc. All rights reserved.
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

#include <cstring> // for std::cerr
#include <errno.h> // for errno on some linux machines
#include <iostream>
#include <string.h> // for memcpy

#include "UdpSocket.h"

// The IP address of the connection
char* UdpSocket::connectionName()
{
    return ip4_;
}

int UdpSocket::read( char* buffer, int length ) const
{
    // UDP receives data using recvfrom() instead of recv()
    struct sockaddr sender_addr;
    socklen_t sender_addr_size = sizeof( sender_addr );
    int result = recvfrom( ndiSocket_, buffer, length, 0, &sender_addr, &sender_addr_size );
    if ( result < 0 )
    {
        std::cout << "UDP recvfrom() read bytes: " << result << " of " << length << std::endl;
    }

    return result;
}

int UdpSocket::read( byte_t* buffer, int length ) const
{
    return read( (char*)buffer, length );
}

bool UdpSocket::connect( const char* hostname )
{
    // Use 54321 as default local port
    return connect( hostname, "54321" );
}

bool UdpSocket::connect( const char* hostname, const char* port )
{
    // Define socket options in the 'addrinfo' block
    addrinfo addressInfo;
    memset( &addressInfo, 0, sizeof( addressInfo ) );
    addressInfo.ai_family = AF_INET;
    addressInfo.ai_socktype = SOCK_DGRAM;
    addressInfo.ai_protocol = IPPROTO_UDP;

    // Setup a UDP socket using the given hostname and port
    addrinfo* aiPointer = NULL, * pai;
    int addrinforesult = getaddrinfo( NULL, port, &addressInfo, &aiPointer );
    if ( addrinforesult != 0 )
    {
        std::cerr << "getaddrinfo Error code " << addrinforesult << " (" << gai_strerror( addrinforesult ) << ")" << std::endl;
        return false;
    }

    for ( pai = aiPointer; pai != NULL; pai = pai->ai_next )
    {
        // Initialize socket. If it doesn't exist, try the next addrinfo in the array.
        ndiSocket_ = ::socket( pai->ai_family, pai->ai_socktype, pai->ai_protocol );
        if ( !socketIsValid() )
            continue;

        // Create the structure to pass to bind()
        struct sockaddr_in sockAddress;
        memset( &sockAddress, 0, sizeof( sockAddress ) );
        sockAddress.sin_family = AF_INET;

        // Using INADDR_ANY binds to 0.0.0.0:port, meaning the port is bound on ALL interface addresses.
        // Computers often have multiple network interfaces (eg. Ethernet ports, wifi, etc...).
        // If we knew the network interface the Vega was connected on, then we could set this explicitly.
        // bind() requires a unique combination of InterfaceIP:Port, so two different sockets can use the same 
        // port if they are on different interfaces.
        sockAddress.sin_addr.s_addr = INADDR_ANY;
        sockAddress.sin_port = htons( atoi( port ) );
        //inet_pton(AF_INET, network_interface_ip4, &sockAddress.sin_addr.s_addr);

        // bind() sets the data source address and port
        // connect() sets the destination address and port, which isn't needed for UDP
        isConnected_ = ::bind( ndiSocket_, (const sockaddr*)&sockAddress, sizeof( sockAddress ) ) >= 0;
        if ( isConnected_ )
        {
            // Convert the IP address to a character array
            inet_ntop( AF_INET, &sockAddress.sin_addr, ip4_, INET_ADDRSTRLEN );
            std::cout << "UdpSocket connected on: " << ip4_ << ":" << port << std::endl;
            break;
        }
        disconnect();
        ndiSocket_ = -1;
    }
    if ( !isConnected_ )
    {
#pragma warning( disable : 4996)
#pragma warning( push )
        std::cerr << "UdpSocket error code " << errno << " (" << std::strerror( errno ) << ")" << std::endl;
        std::cerr << "Please ensure you have disabled firewalls, especially for error 48 or 49" << std::endl;
        // TODO: how to set the firewall exception for the UDP port rather than disabling it entirely? 
#pragma warning( pop )
    }
    freeaddrinfo( aiPointer );

    return isConnected_;
}

bool UdpSocket::socketIsValid() const
{
#ifdef _WIN32
    // On Windows, SOCKET type is unsigned, so a negative number cannot indicate invalid socket
    // so "all bits set" indicates invalid sockets:  #define INVALID_SOCKET (SOCKET)(~0)
    return ndiSocket_ != INVALID_SOCKET;
#else
    // On Mac/Linux, sockets are simple. If the file descriptor is negative, the socket is invalid
    return ndiSocket_ >= 0;
#endif
}

void UdpSocket::disconnect()
{
#ifdef _WIN32
    closesocket( ndiSocket_ );
#else
    close( ndiSocket_ );
#endif
}

void UdpSocket::init()
{
#ifdef _WIN32
    // Initializes WinSock using the v2 implementation
    // See: https://msdn.microsoft.com/en-us/library/windows/desktop/ms738545%28v=vs.85%29.aspx
    WSAStartup( MAKEWORD( 2, 2 ), &wsaData_ );
#endif
    ndiSocket_ = 0;
    isConnected_ = false;
    ip4_[0] = 0;
    protocol = StreamingProtocol::UDP;
}

bool UdpSocket::isConnected() const
{
    return isConnected_;
}

UdpSocket::UdpSocket( const char* hostname, const char* port )
{
    init();
    connect( hostname, port );
}

UdpSocket::UdpSocket()
{
    init();
}

UdpSocket::~UdpSocket()
{
    disconnect();

#ifdef _WIN32
    // An application must call the WSACleanup function for every successful time the WSAStartup function is called.
    WSACleanup();
#endif
}
