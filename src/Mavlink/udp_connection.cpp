#include "udp_connection.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>

namespace mavlink {

UdpConnection::UdpConnection(const std::string& local_ip, int local_port_number, std::function<void(const mavlink_message_t& message)> message_callback)
	: _local_ip(local_ip)
	, _local_port_number(local_port_number)
	, _message_handler(message_callback)
{}

UdpConnection::~UdpConnection()
{
	stop();
}

ConnectionResult UdpConnection::start()
{
	ConnectionResult ret = setup_port();
	if (ret != ConnectionResult::Success) {
		return ret;
	}

	// TESTING:
	// By default we try and talk to a GCS on port 14550
	// add_remote("127.0.0.1", 14550);

	start_recv_thread();

	return ConnectionResult::Success;
}

ConnectionResult UdpConnection::stop()
{
	_should_exit = true;

	shutdown(_socket_fd, SHUT_RDWR);
	close(_socket_fd);

	if (_recv_thread) {
		_recv_thread->join();
		delete _recv_thread;
		_recv_thread = nullptr;
	}

	return ConnectionResult::Success;
}

ConnectionResult UdpConnection::setup_port()
{
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		LOG("socket error");
		return ConnectionResult::SocketError;
	}

	struct sockaddr_in addr = {};
	addr.sin_family = AF_INET;
	inet_pton(AF_INET, _local_ip.c_str(), &(addr.sin_addr));
	addr.sin_port = htons(_local_port_number);

	if (bind(_socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
		LOG("bind error");
		return ConnectionResult::BindError;
	}

	return ConnectionResult::Success;
}

void UdpConnection::start_recv_thread()
{
	_recv_thread = new std::thread(&UdpConnection::receive, this);
}

bool UdpConnection::send_message(const mavlink_message_t& message)
{
	std::lock_guard<std::mutex> lock(_remote_mutex);

	if (_remotes.size() == 0) {
		LOG("No known remotes");
		return false;
	}

	// Send the message to all the remotes.
	bool send_successful = true;

	for (auto& remote : _remotes) {

		struct sockaddr_in dest_addr {};
		dest_addr.sin_family = AF_INET;

		inet_pton(AF_INET, remote.ip.c_str(), &dest_addr.sin_addr.s_addr);
		dest_addr.sin_port = htons(remote.port_number);

		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
		uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

		const auto send_len = sendto(_socket_fd, reinterpret_cast<char*>(buffer), buffer_len,
			0, reinterpret_cast<const sockaddr*>(&dest_addr), sizeof(dest_addr));

		if (send_len != buffer_len) {
			LOG("sendto failure");
			send_successful = false;
			continue;
		}
	}

	return send_successful;
}

void UdpConnection::add_remote(const std::string& remote_ip, const int remote_port)
{
	add_remote_with_remote_sysid(remote_ip, remote_port, 0);
}

void UdpConnection::add_remote_with_remote_sysid( const std::string& remote_ip, const int remote_port, const uint8_t remote_sysid)
{
	std::lock_guard<std::mutex> lock(_remote_mutex);

	Remote new_remote;
	new_remote.ip = remote_ip;
	new_remote.port_number = remote_port;

	auto existing_remote =
		std::find_if(_remotes.begin(), _remotes.end(), [&new_remote](Remote& remote) {
			return remote == new_remote;
		});

	if (existing_remote == _remotes.end()) {
		LOG("New system on: %s:%d (with sysid: %d)", new_remote.ip.c_str(), new_remote.port_number, remote_sysid);
		_remotes.push_back(new_remote);
	}
}

void UdpConnection::receive()
{
	// Enough for MTU 1500 bytes.
	char buffer[2048];

	while (!_should_exit) {

		struct sockaddr_in src_addr = {};
		socklen_t src_addr_len = sizeof(src_addr);

		const auto recv_len = recvfrom(
			_socket_fd,
			buffer,
			sizeof(buffer),
			0,
			reinterpret_cast<struct sockaddr*>(&src_addr), &src_addr_len);

		if (recv_len == 0) {
			// This can happen when shutdown is called on the socket, therefore we check _should_exit again.
			continue;
		}

		if (recv_len < 0) {
			// This happens on destruction when close(_socket_fd) is called, therefore be quiet.
			LOG("recvfrom error");
			continue;
		}

		// Set new datagram
		_datagram = buffer;
		_datagram_len = recv_len;

		bool saved_remote = false;

		// Parse all mavlink messages in one datagram. Once exhausted, we'll exit while.
		while (parse_message()) {

			const uint8_t sysid = _last_message.sysid;
			const uint32_t msgig = _last_message.msgid;

			if (!saved_remote && sysid != 0) {
				saved_remote = true;
				add_remote_with_remote_sysid(inet_ntoa(src_addr.sin_addr), ntohs(src_addr.sin_port), sysid);
			}

			switch (msgig) {
				case MAVLINK_MSG_ID_HEARTBEAT:

					// Check last received heartbeat time
					uint64_t time_now = millis();
					bool timed_out = time_now > _last_heartbeat_ms + CONNECTION_TIMEOUT_MS;

					if (timed_out) {
						LOG("Connected to System ID: " GREEN_TEXT "%u" NORMAL_TEXT, sysid);
						// TODO: we could make a thread/timer that keeps track
						// of when systems connect/timeout.
					}

					_last_heartbeat_ms = time_now;
					break;
			}

			// Call the message handler callback
			_message_handler(_last_message);
		}
	}
}

bool UdpConnection::parse_message()
{
	// Note that one datagram can contain multiple mavlink messages.
	for (unsigned i = 0; i < _datagram_len; ++i) {
		if (mavlink_parse_char(_channel, _datagram[i], &_last_message, &_status) == 1) {
			// Move the pointer to the datagram forward by the amount parsed.
			_datagram += (i + 1);
			// And decrease the length, so we don't overshoot in the next round.
			_datagram_len -= (i + 1);
			// We have parsed one message, let's return so it can be handled.
			return true;
		}
	}

	// No (more) messages, let's give up.
	_datagram = nullptr;
	_datagram_len = 0;
	return false;
}

} // end namespace mavlink
