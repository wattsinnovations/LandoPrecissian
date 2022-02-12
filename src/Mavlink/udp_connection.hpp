#pragma once

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <functional>

#include <time.h>

#include <mavlink.h>

#include <global_include.hpp>

namespace mavlink {

static constexpr uint64_t CONNECTION_TIMEOUT_MS = 2000;

enum class ConnectionResult {
	Success = 0,
	Timeout,
	SocketError,
	BindError,
	SocketConnectionError,
	ConnectionError,
	NotImplemented,
	SystemNotConnected,
	SystemBusy,
	CommandDenied,
	DestinationIpUnknown,
	ConnectionsExhausted,
	ConnectionUrlInvalid,
	BaudrateUnknown
};

class UdpConnection {
public:
	UdpConnection(const std::string& local_ip, int local_port, std::function<void(const mavlink_message_t& message)> message_callback);

	~UdpConnection();
	ConnectionResult start();
	ConnectionResult stop();

	bool connected() const {
		uint64_t time_now = millis();
		return time_now < _last_heartbeat_ms + CONNECTION_TIMEOUT_MS;
	};

	bool send_message(const mavlink_message_t& message);

	// Non-copyable
	UdpConnection(const UdpConnection&) = delete;
	const UdpConnection& operator=(const UdpConnection&) = delete;

private:
	ConnectionResult setup_port();
	void start_recv_thread();

	void receive();
	bool parse_message();

	void add_remote(const std::string& remote_ip, const int remote_port);
	void add_remote_with_remote_sysid(const std::string& remote_ip, const int remote_port, const uint8_t remote_sysid);

	std::string _local_ip;
	int _local_port_number;

	struct Remote {
		std::string ip;
		int port_number = 0;

		bool operator==(const UdpConnection::Remote& other)
		{
			return ip == other.ip && port_number == other.port_number;
		}
	};

	std::mutex _remote_mutex{};
	std::vector<Remote> _remotes{};

	int _socket_fd = -1;
	std::thread* _recv_thread = nullptr;
	std::atomic_bool _should_exit{false};

	// Mavlink internal data
	char* _datagram = nullptr;
	unsigned _datagram_len = 0;

	uint8_t _channel = 0;
	mavlink_message_t _last_message = {};
	mavlink_status_t _status = {};

	uint64_t _last_heartbeat_ms = 0;

	std::function<void(const mavlink_message_t& message)> _message_handler;
};

} // end namespace mavlink
