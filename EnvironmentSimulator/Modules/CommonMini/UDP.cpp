/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include <stdio.h>
#include <errno.h>
#include <limits.h>

#ifndef _WIN32
#include <sys/time.h>
#endif

#include "UDP.hpp"
#include "CommonMini.hpp"
#include "logger.hpp"

#if defined(__APPLE__)
#include <sys/types.h>
#include <sys/sysctl.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <netinet/udp_var.h>
#endif

UDPBase::UDPBase(unsigned short int port) : port_(port), sock_(SE_INVALID_SOCKET)
{
    sender_addr_size_ = sizeof(sender_addr_);

#ifdef _WIN32
    WSADATA wsa_data;
    int     iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (iResult != NO_ERROR)
    {
        wprintf(L"WSAStartup failed with error %d\n", iResult);
        return;
    }
#endif

    if ((sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SE_INVALID_SOCKET)
    {
        LOG_ERROR_AND_QUIT("socket failed");
        return;
    }
}

int UDPBase::Bind(struct sockaddr_in& addr)
{
    int retval = bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (retval != 0)
    {
#ifdef _WIN32
        LOG_ERROR("bind error {}", WSAGetLastError());
#else
        perror("bind socket");
        LOG_ERROR("bind socket error");
#endif
        CloseGracefully();
        LOG_ERROR_AND_QUIT("Bind UDP socket on port {} failed (return code {})", port_, retval);
        return -1;
    }

    return 0;
}

void UDPBase::CloseGracefully()
{
#ifdef _WIN32
    if (closesocket(sock_) == SOCKET_ERROR)
#else
    if (close(sock_) < 0)
#endif
    {
#ifdef _WIN32
        LOG_ERROR("Failed closing socket {}", WSAGetLastError());
#else
        perror("close socket");
        LOG_ERROR("close socket error");
#endif
    }

#ifdef _WIN32
    WSACleanup();
#endif
}

UDPServer::UDPServer(unsigned short int port, unsigned int timeoutMs) : UDPBase(port), timeoutMs_(timeoutMs)
{
    // set timer for receive operations
#ifdef _WIN32
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs_, sizeof(timeoutMs_)) != 0)
#else
    struct timeval tv;
    tv.tv_sec = timeoutMs_ / 1000;
    tv.tv_usec = (timeoutMs_ % 1000) * 1000;
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
#endif
    {
        LOG_WARN("socket SO_RCVTIMEO (receive timeout) not supported on this platform");
    }

    // set re-use address option
    unsigned int reuse_addr = 1;  // 1 means ON, 0 means OFF
#ifdef _WIN32
    if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse_addr, sizeof(reuse_addr)) != 0)
#else
    if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) < 0)
#endif
    {
        LOG_WARN("socket SO_REUSEADDR not supported on this platform");
    }

    // Prepare the sockaddr_in structure
    server_addr_.sin_family      = AF_INET;
    server_addr_.sin_port        = htons(port_);
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

    Bind(server_addr_);
}

int UDPServer::Receive(char* buf, unsigned int size)
{
    // TODO:
    // Casting to int can cause overflow in this situation. Not a good idea.
    // Let's fix it in a way that we actually return size_t and design the flow like that
    return static_cast<int>(recvfrom(sock_, buf, size, 0, reinterpret_cast<struct sockaddr*>(&sender_addr_), &sender_addr_size_));
}

UDPClient::UDPClient(unsigned short int port, std::string ipAddress) : UDPBase(port), ipAddress_(ipAddress)
{
    // Prepare the sockaddr_in structure
    memset(reinterpret_cast<char*>(&server_addr_), 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port   = htons(port_);
    inet_pton(AF_INET, ipAddress.c_str(), &server_addr_.sin_addr.s_addr);
}

int UDPClient::Send(char* buf, unsigned int size)
{
    if (size > GetMaxUDPDatagramSize())
    {
        LOG_ERROR("Attempting to send UDP datagram of size {} which exceeds system maximum datagram size {}", size, GetMaxUDPDatagramSize());
        return -1;
    }
    // Here, auto handles different signatures of sendto
    const auto sent_bytes = sendto(sock_, buf, size, 0, reinterpret_cast<struct sockaddr*>(&server_addr_), sizeof(server_addr_));
    if (sent_bytes < 0)
    {
#ifdef _WIN32
        LOG_ERROR("sendto failed with WinSOCK error number: {}", WSAGetLastError());
#else
        LOG_ERROR("sendto failed with error: {}", strerror(errno));
#endif
        return -1;
    }
    return static_cast<int>(sent_bytes);
}

unsigned int UDPClient::GetMaxUDPDatagramSize()
{
#if defined(__APPLE__)
    // NOTE:
    // C++17 Trick (available for Mac Builds), lambda directly called on static to avoid
    // recalling sysctlbyname every time we need to send something over UDP.
    // We are caching it the return value in a static, initialized once per C++17 standard.
    static const unsigned int max_dgram = []
    {
        size_t len = sizeof(int);
        int    max_dgram_;
        if (sysctlbyname("net.inet.udp.maxdgram", &max_dgram_, &len, nullptr, 0) == 0)
        {
            return static_cast<unsigned int>(max_dgram_);
        }
        return 9216;  // Default value fallback
    }();
    return max_dgram;
#else
    return 65507;  // Default value for Windows and Linux
#endif
}