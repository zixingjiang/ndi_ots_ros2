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

#include <iostream>
#include <string.h>

#include <CryptoConnection.h>

// Initialize to false on startup, and set to true when the one-time library init() has run
bool CryptoConnection::cryptoInitialized = false;

void CryptoConnection::print_cn_name(const char* label, X509_name_st* const name)
{
    // No name
    if (!name) return;

    // Get the index of "Common Name (CN)"
    int index = X509_NAME_get_index_by_NID(name, NID_commonName, -1);
    if (index < 0) return;

    // Get the name entry by index
    X509_NAME_ENTRY * entry = X509_NAME_get_entry(name, index);
    if (!entry) return;

    // Get a pointer to the name data
    ASN1_STRING * data = X509_NAME_ENTRY_get_data(entry);
    if (!data) return;

    // Get a pointer to the name as a UTF8 string
    unsigned char* utf8 = NULL;
    int length = ASN1_STRING_to_UTF8(&utf8, data);
    if (!utf8 || length <= 0)  return;

    // Print the string to stdout
    std::cout << label << utf8 << std::endl;

    // Free the UTF8 string
    OPENSSL_free(utf8);
}

int CryptoConnection::verify_callback(int verifyResult, X509_STORE_CTX * x509_ctx)
{
    int depth = X509_STORE_CTX_get_error_depth(x509_ctx);
    int err = X509_STORE_CTX_get_error(x509_ctx);

    // Print some information from the server certificate
    // Issuer: the authority we trust
    // Subject: who the certificate is issued to by the authority
    X509* cert = X509_STORE_CTX_get_current_cert(x509_ctx);
    X509_name_st* iname = cert ? X509_get_issuer_name(cert) : NULL;
    X509_name_st* sname = cert ? X509_get_subject_name(cert) : NULL;
    print_cn_name("Issuer CN: ", iname);
    print_cn_name("Subject CN: ", sname);

    if (verifyResult == 0)
    {
        switch (err)
        {
        case X509_V_ERR_UNABLE_TO_GET_ISSUER_CERT_LOCALLY:
            std::cout << "X509_V_ERR_UNABLE_TO_GET_ISSUER_CERT_LOCALLY: this occurs if the issuer certificate of an untrusted certificate cannot be found." << std::endl;
            break;
        case X509_V_ERR_CERT_UNTRUSTED:
            std::cout << "X509_V_ERR_CERT_UNTRUSTED: the root CA is not marked as trusted for the specified purpose." << std::endl;
            break;
        case X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN:
            std::cout << "X509_V_ERR_SELF_SIGNED_CERT_IN_CHAIN: the certificate chain could be built up using the untrusted certificates but the root could not be found locally." << std::endl;
            break;
        case X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT:
            std::cout << "X509_V_ERR_DEPTH_ZERO_SELF_SIGNED_CERT: the passed certificate is self signed and the same certificate cannot be found in the list of trusted certificates." << std::endl;
            verifyResult = 1;
            // TODO: Allow self-signed certificates for development/debugging. Use verifyResult=0 for production to forbid self-signed production certificates.
            break;
        case X509_V_ERR_CERT_NOT_YET_VALID:
            std::cout << "X509_V_ERR_CERT_NOT_YET_VALID: the notBefore date is after the current time." << std::endl;
            break;
        case X509_V_ERR_CERT_HAS_EXPIRED:
            std::cout << "X509_V_ERR_CERT_HAS_EXPIRED: the notAfter date is before the current time." << std::endl;
            break;
        case X509_V_OK:
            std::cout << "X509_V_OK: the operation was successful" << std::endl;
            verifyResult = 1;
            break;
        default:
            std::cout << "Error: " << err << std::endl;
        }
    }

    return verifyResult;
};

void CryptoConnection::disconnect()
{
    // Cleanup BIO
    if (ssl_bio_ != nullptr)
    {
        BIO_free_all(ssl_bio_);
        ssl_bio_ = nullptr;
    }

    // Cleanup SSL CTX
    if (ssl_ctx_ != nullptr)
    {
        SSL_CTX_free(ssl_ctx_);
        ssl_ctx_ = nullptr;
    }
}

bool CryptoConnection::isConnected() const
{
    return isConnected_;
}

int CryptoConnection::write(const char* buffer, int length) const
{
    int result = SSL_write(ssl_, buffer, length);
    //std::cout << "------------BIO write: " << (result > 0 ? "succeeded" : "failed") << std::endl;
    return result;
}

int CryptoConnection::write(byte_t* buffer, int length) const
{
    return write((const char*)buffer, length);
}

char* CryptoConnection::connectionName()
{
    return (char*)connectionName_.c_str();
}

int CryptoConnection::read(char* buffer, int length) const
{
    int result = SSL_read(ssl_, buffer, length);
    //std::cout << "------------BIO read: " << (result > 0 ? "succeeded" : "failed") << std::endl;
    return result;
}

int CryptoConnection::read(byte_t* buffer, int length) const
{
    return read((char*)buffer, length);
}

void CryptoConnection::init()
{
    // OpenSSL library initialization should occur exactly once!
    if (!cryptoInitialized)
    {
        // OpenSSL's version number looks like:  0x00090605f == 0.9.6e release
        // Header files contain macros for the version you linked against: OPENSSL_VERSION_TEXT
        std::cout << "Sample was compiled linking against shared library: " << OPENSSL_VERSION_TEXT << std::endl;
        std::cout << "Sample actually loaded shared libary: " << OpenSSL_version(OPENSSL_VERSION) << std::endl;

        // Hardcode a minimum version of OpenSSL required for this code to run and operate securely
        if (OPENSSL_VERSION_NUMBER < 0x1010100fL) {
            std::cout << "Unsupported OpenSSL version! Use version 1.1.1 or higher\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        else
        {
            std::cout << "OpenSSL version meets the minimum requirement (1.1.1a)" << std::endl;
        }

        // Check the shared library linked at runtime is compatible
        if (OpenSSL_version_num() != OPENSSL_VERSION_NUMBER)
        {
            std::cout << "Warning: OpenSSL version mismatch!\n" << std::endl;
            std::cout << "Linked against: " << OpenSSL_version(OPENSSL_VERSION) << std::endl;

            // Major.minor version must match, different patch versions of the same library are acceptable
            if (OpenSSL_version_num() >> 20 != OPENSSL_VERSION_NUMBER >> 20)
            {
                std::cout << "Error: Major and minor version numbers must match, exiting.\n" << std::endl;
                exit(EXIT_FAILURE);
            }
            std::cout << "Continuing with: " << OpenSSL_version(OPENSSL_VERSION) << std::endl;
        }

        // Initialize the cryptographic library
        // To configure non-default options on initialization, use: OPENSSL_init_ssl()
        // See: https://www.openssl.org/docs/man1.1.1/man3/OPENSSL_init_ssl.html
        // OpenSSL v1.1.0 and later provide default initialization, and do not require these calls:
        // SSL_library_init();
        // SSL_load_error_strings();

#if defined (OPENSSL_THREADS)
        // TODO: OpenSSL - do we need OpenSSL threading support?
        std::cout << "OpenSSL threading support is available" << std::endl;
#endif

        // Set a flag to ensure the library SSL_library_init() is only called once
        cryptoInitialized = true;
    }
}

bool CryptoConnection::connect(const char* hostname, const char* port, const char* cipher, SecureProtocol protocol)
{
    // Secure connection is not possible if hostname is blank
    if (std::string(hostname).empty())
    {
        std::cerr << "TLS connection failed: hostname is blank." << std::endl;
        isConnected_ = false;
        return false;
    }

    // Remember to check error codes on each library call!
    unsigned long ssl_err = 0ul;
    long ssl_response = 0l;

    // TLS_method() negotaties the most recent protocol version supported by both server and client
    const SSL_METHOD* method = negotiateProtocol();
    ssl_err = ERR_get_error();
    if (method == nullptr)
    {
        std::cerr << "TLS protocol negotiation failed." << std::endl;
        return false;
    }

    // Setup an SSL context
    ssl_ctx_ = SSL_CTX_new(method);
    ssl_err = ERR_get_error();
    if (ssl_ctx_ == nullptr)
    {
        std::cerr << "TLS context creation failed." << std::endl;
        return false;
    }

    // Provide a callback function to OpenSSL library for certificate verification.
    // Pass NULL to use OpenSSL's default certificate verification, if desired.
    // Note: If you pass NULL, the BIO will fail to connect if there are certificate errors, and it won't be obvious why it fails!
    SSL_CTX_set_verify(ssl_ctx_, SSL_VERIFY_PEER, verify_callback);

    // Set a bunch of flags that remove insecure protocols, and add various bug fixes and workarounds
    const long flags = SSL_OP_ALL | SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3 | SSL_OP_NO_COMPRESSION;
    long old_opts = SSL_CTX_set_options(ssl_ctx_, flags);

    // Default behaviour
    if(strcmp(cipher, "") == 0)
    {
        // Disable ciphers that are known to be insecure, leaving only high security ciphers enabled
        ssl_response = SSL_CTX_set_cipher_list(ssl_ctx_, PREFERRED_CIPHERS);
    }
    // Runs when a cipher has been specified
    else
    {
        bool v1Dot3 = false;
        for(int i = 0; i < NUM_1DOT3_CIPHERS; i++)
        {
            // Check if the cipher is one of the TLSv1.3 ciphersuites
            if(strstr(cipher, TLS_1DOT3_CIPHERS[i].c_str()) != nullptr)
            {
                v1Dot3 = true;
            }
        }

        if(protocol == SecureProtocol::DTLS)
        {
            if(v1Dot3)
            {
                // Note that all ciphersuites used by DTLS are also used by TLS but some ciphersuites used by TLS cannot be used
                // by DTLS. As of OpenSSL 3.0.0, support for DTLSv1.3 has not been implemented yet. This is contrary to TLSv1.3,
                // which support was added for in OpenSSL 1.1.1.
                std::cerr << "TLS protocol configuration failed. OpenSSL does not support DTLSv1.3 ciphersuites." << std::endl;
                return false;
            }
            else
            {
                // Set the protocol version to DTLSv1.2 and below to eliminate use of DTLSv1.3 (if ever implemented in OPENSSL)
                ssl_response = SSL_CTX_set_max_proto_version(ssl_ctx_, DTLS1_2_VERSION);

                // Check that setting the max protocol version was successful
                if (ssl_response != 1)
                {
                    std::cerr << "TLS protocol configuration failed." << std::endl;
                    return false;
                }

                // Set the ciphersuite for DTLSv1.2 and below
                ssl_response = SSL_CTX_set_cipher_list(ssl_ctx_, cipher);
            }
        }
        else // SecureProtocol::TLS
        {
            if(v1Dot3)
            {
                // Set the protocol version to TLSv1.3 by making TLSv1.3 the min and max protocol
                ssl_response = SSL_CTX_set_min_proto_version(ssl_ctx_, TLS1_3_VERSION);
                if (ssl_response != 1)
                {
                    std::cerr << "TLS protocol configuration failed." << std::endl;
                }
                ssl_response = SSL_CTX_set_max_proto_version(ssl_ctx_, TLS1_3_VERSION);

                // Check that setting the max protocol version was successful
                if (ssl_response != 1)
                {
                    std::cerr << "TLS protocol configuration failed." << std::endl;
                }

                // Set the ciphersuite for TLSv1.3 if the cipher is a TLSv1.3 ciphersuite
                ssl_response = SSL_CTX_set_ciphersuites(ssl_ctx_, cipher);
            }
            else
            {
                // Set the protocol version to TLSv1.2 and below to eliminate use of TLSv1.3
                 ssl_response = SSL_CTX_set_max_proto_version(ssl_ctx_, TLS1_2_VERSION);

                // Check that setting the max protocol version was successful
                if (ssl_response != 1)
                {
                    std::cerr << "TLS protocol configuration failed." << std::endl;
                }

                // Set the ciphersuite for TLSv1.2 and below if the ciphersuite is not a TLSv1.3 ciphersuite
                ssl_response = SSL_CTX_set_cipher_list(ssl_ctx_, cipher);
            }
        }
    }

    if (ssl_response != 1)
    {
        std::cerr << "TLS cipher configuration failed." << std::endl;
        // Using default ciphers may be less secure, but it is okay for the purposes of this sample app.
        // In your application, you must decide if you want to fail here.
    }

    // TODO: OpenSSL needs to know CA information if a signed certificate is used (ie. NOT a self-signed certificate)
    // SSL_CTX_load_verify_locations(ctx, CAFile*, CAPath*) verifies the server's public certificate chain
    // ssl_response = SSL_CTX_load_verify_locations(ssl_ctx_, "NDI-devcert.pem", NULL);
    // ssl_err = ERR_get_error();
    // if (ssl_response != 1)
    // {
    //    std::cerr << "SSL certificate verification failed." << std::endl;
    // }

    // The BIO must be provided the hostname and port, and set the connectionName() property
    // BIOs appear to be smart enough to determine the IP address from the hostname, as in: "P9-C0041.local:8764"
    connectionName_ = std::string(hostname).append(":").append(port);

    // OpenSSL Basic Input/Output (BIO) blocks are used to chain together complex operations (like GStreamer components)
    ssl_bio_ = createBIO();
    if (ssl_bio_ == nullptr)
    {
        std::cerr << "TLS Basic Input/Output (BIO) creation failed." << std::endl;
        return false;
    }

    // Try to connect, and return the result
    isConnected_ = doConnect();
    return isConnected_;
}

char* CryptoConnection::connectionIP()
{
    int getNumericAddress = 1; // eg. "169.254.11.151";
    const BIO_ADDR* serverAddress = BIO_get_conn_address(ssl_bio_);
    if (serverAddress == nullptr)
    {
        // BIO_get_conn_address() returns null if no address was set. It is set when connecting the BIO.
        // See: https://www.openssl.org/docs/man1.1.1/man3/BIO_get_conn_address.html
        return nullptr;
    }
    return BIO_ADDR_hostname_string(serverAddress, getNumericAddress);

}

#endif  // #ifdef OPENSSL