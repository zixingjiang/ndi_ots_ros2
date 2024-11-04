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

#ifndef CRYPTO_CONNECTION_HPP
#define CRYPTO_CONNECTION_HPP

#ifdef _WIN32
// Link the OpenSSL dynamic libraries on Windows
// This is done in Makefile compiler commands for Mac/Linux
#pragma comment(lib, "libssl.lib")
#pragma comment(lib, "libcrypto.lib")
#endif

#include <string>

// TLS/DTLS sockets for encrypted TCP/UDP
#include <openssl/bio.h>         // for BIOs
#include <openssl/conf.h>        // for configuration options
#include <openssl/err.h>         // for error strings
#include <openssl/opensslconf.h> // for threading support
#include <openssl/opensslv.h>    // for OpenSSL version
#include <openssl/ssl.h>         // for libssl functions
#include <openssl/x509.h>        // for x509 certificate validation functions
#include <openssl/x509v3.h>

#include <Connection.h>

enum class SecureProtocol { TLS, DTLS };

class CryptoConnection : public Connection
{
public:

    // Disable ciphers that are known to be insecure, leaving only high security ciphers enabled
    const char* const PREFERRED_CIPHERS = "HIGH:!aNULL:!eNULL:!ADH:!MD5:!RC4:!SRP:!PSK:!DSS:!SSLv3:!SSLv2:!TLSv1";

    static const int NUM_1DOT3_CIPHERS = 5;
    const std::string TLS_1DOT3_CIPHERS[ NUM_1DOT3_CIPHERS ] =
    {"TLS_AES_128_GCM_SHA256",
     "TLS_AES_256_GCM_SHA384",
     "TLS_CHACHA20_POLY1305_SHA256",
     "TLS_AES_128_CCM_SHA256",
     "TLS_AES_128_CCM_8_SHA256"
    } ;

    /*
     * @brief Prints the certificate name
     * @param label The label to print before the certificate name
     * @param name The X509 certificate name object
     */
    static void print_cn_name(const char* label, X509_name_st* const name);

    /*
     * @brief OpenSSL callback for certificate verification
     * @param verifyResult The result of certificate verification performed by the OpenSSL library
     * @param x509_ctx The X509 certificate chain presented by the server to validate
     * @return Return 1 to accept the certificate, or 0 to fail the certificate verification
     */
    static int verify_callback(int verifyResult, X509_STORE_CTX* x509_ctx);

protected:

    //! Subclasses provide the openssl method to negotiate the protocol. Eg. TLS_method() vs. DTLS_method().
    virtual const SSL_METHOD* negotiateProtocol() = 0;

    //! Creates the Basic Input/Output (BIO) for the protocol
    virtual BIO* createBIO() = 0;

    /*
     * @brief Subclasses provide connection code specific to their protocol.
     * @return True for a successful connection, false otherwise.
     */
    virtual bool doConnect() = 0;

    /**
     * @brief Closes any existing connection, and connects to the new device.
     * @param hostname The hostname or IP address of the device.
     * @param port The port number to connect on.
     */
    bool connect(const char* hostname, const char* port, const char* cipher, SecureProtocol protocol);

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

    //! Returns the IP address of the connection, or nullptr if the BIO is not connected
    char* connectionIP();

    //! @copydoc Connection::connectionName()
    char* connectionName();

    //! One-time setup of cryptography library
    void init();

    //! Indicates whether the cryptography libraries one-time setup has occurred already
    static bool cryptoInitialized;

    //! True if the BIO is ready to send/recv
    bool isConnected_ = false;

    //! The hostname accepts either hostname:port or IPaddress:port formats
    std::string connectionName_;

    //! OpenSSL Basic Input/Output (BIO) pipeline for encrypted communication
    BIO* ssl_bio_ = nullptr;

    //! OpenSSL context used to set options that control the connection
    SSL_CTX* ssl_ctx_ = nullptr;

    //! OpenSSL connection object used for SSL_read() and SSL_write()
    SSL* ssl_ = nullptr;
};

#endif // CRYPTO_CONNECTION_HPP