#ifndef MINIMESH_HPP
#define MINIMESH_HPP
#include <cinttypes>
#include "bytes.hpp"

namespace minimesh
{
    /* -------------------------------------------------------------------------- */
    /*                               User Interface                               */
    /* -------------------------------------------------------------------------- */
    using Id = uint32_t;
    using ReceiveFunc = auto(uint32_t timeout_ms) -> Bytes;
    using TransmitFunc = auto(ConstBytes bytes) -> void;
    using SleepFunc = auto(uint32_t duration_us) -> void;
    using IsChannelBusyFunc = auto() -> bool;
    using CollectorCallback = auto(Id device_id, ConstBytes data) -> void;
    constexpr CollectorCallback *no_callback =
        reinterpret_cast<CollectorCallback *>(NULL);

    template <ReceiveFunc *receive, TransmitFunc *transmit, SleepFunc *sleep,
              IsChannelBusyFunc *is_channel_busy, Id id, uint32_t data_length,
              bool is_collector, CollectorCallback *collector_callback = no_callback>
    struct Handle
    {

        auto run() const -> void
        {
            if constexpr (is_collector)
            {
                run_as_collector();
            }
            else
            {
                run_as_sensor();
            }
        }

        auto get_data_buffer() const -> uint8_t *
        {
            static_assert(!is_collector, "data buffer is only used for sensors");
            return data_packet->data;
        }

        ;
        /* -------------------------------------------------------------------------- */
        /*                           Implementation Details                           */
        /* -------------------------------------------------------------------------- */
    private:
        enum MsgType : uint32_t
        {
            IAmParent,
            IAmChild,
            Data,
            EndOfData,
            Ack,
        };
        static constexpr uint32_t max_packet_size = 255;
        static constexpr uint32_t header_size = sizeof(MsgType) + sizeof(uint32_t) + sizeof(uint32_t);
        static constexpr uint32_t max_data_length = max_packet_size - header_size;
        static constexpr auto sleep_time = (id % 9000) + 1000;
        static constexpr Id broadcast = 0;
        static_assert(data_length < max_packet_size, "Data cannot be longer than 255 bytes");
        struct Packet
        {
            MsgType msg_type;        // Type of message
            uint32_t transmitter_id; // Id of transmitting device
            uint32_t receiver_id;    // Id of intended receiver (0 means broadcast)
            uint8_t data[];          // Custom data
        };
        struct Header
        {
            MsgType msg_type;        // Type of message
            uint32_t transmitter_id; // Id of transmitting device
            uint32_t receiver_id;    // Id of intended receiver (0 means broadcast)
            operator ConstBytes() const
            {
                return {reinterpret_cast<const uint8_t *>(this), header_size};
            }
            operator Bytes()
            {
                return {reinterpret_cast<uint8_t *>(this), header_size};
            }
            operator Packet()
            {
                return *reinterpret_cast<Packet *>(this);
            }
        };
        struct ConstPacketWrapper
        {
            const Packet *packet;
            const uint32_t length;
        };
        struct PacketWrapper
        {
            Packet *packet;
            uint32_t length;
            operator ConstPacketWrapper() const
            {
                return {packet, length};
            }
        };
        enum Result
        {
            Fail,
            Ok,
        };
        Packet *data_packet = []()
        {
            static uint8_t buffer[header_size + data_length];
            auto packet = reinterpret_cast<Packet *>(buffer);
            packet->msg_type = MsgType::Data;
            packet->transmitter_id = id;
            return packet;
        }();
        auto run_as_sensor() const -> void
        {
            const auto parent_id = find_parent();
            const auto child_count = count_children();
            proxy_children(parent_id, child_count);
            send_own_data(parent_id);
            send_end_of_data(parent_id);
        };
        auto run_as_collector() const -> void
        {
            auto child_count = count_children();
            while (child_count > 0)
            {
                const auto [packet, length] = receive_packet(5000);
                if (length == 0)
                    return;
                if (packet->receiver_id != id)
                    continue;
                if (packet->msg_type == MsgType::EndOfData)
                {
                    child_count--;
                    send_ack(packet->transmitter_id);
                }

                if (packet->msg_type == MsgType::Data)
                {
                    collector_callback(packet->transmitter_id, {reinterpret_cast<uint8_t *>(packet),
                                                                length});
                    send_ack(packet->transmitter_id);
                }
            }
        }
        auto find_parent() const -> Id
        {
            auto packet_wrapper = receive_packet(0);
            auto [packet, length] = packet_wrapper;
            if (packet->msg_type != MsgType::IAmParent)
                return find_parent(); // try again
            const auto parent_id = packet->transmitter_id;
            const Packet i_am_child = {
                MsgType::IAmChild,
                id,
                parent_id,
            };
            const ConstPacketWrapper i_am_child_wrapper = {
                &i_am_child,
                header_size,
            };
            if (deliver(i_am_child_wrapper))
                return parent_id;
            return find_parent(); // try again
        }
        auto count_children() const -> uint32_t
        {
            transmit_i_am_parent();
            auto child_count = 0;
            while (true)
            {
                const auto [packet, length] = receive_packet(100);
                if (length == 0)
                    return child_count;
                if (packet->receiver_id == id && packet->msg_type == MsgType::IAmChild)
                {
                    send_ack(packet->transmitter_id);
                    child_count++;
                }
            }
        }
        auto proxy_children(Id parent_id,
                            uint32_t child_count) const -> void
        {
            if (child_count == 0)
                return;
            auto [packet, length] = receive_packet(5000);
            if (length == 0)
                return;
            if (packet->receiver_id != id)
                return proxy_children(parent_id, child_count); // Try again
            if (packet->msg_type != MsgType::Data && packet->msg_type != MsgType::EndOfData)
                return proxy_children(parent_id, child_count); // Try again
            send_ack(packet->transmitter_id);
            if (packet->msg_type == MsgType::EndOfData)
                return proxy_children(parent_id, child_count - 1); // One child done
            packet->receiver_id = parent_id;
            deliver({packet,
                     length});
        }
        auto deliver(ConstPacketWrapper packet_wrapper) const -> Result
        {
            for (auto attempts = 0; attempts < 10; attempts++)
            {
                sleep(sleep_time);
                while (is_channel_busy())
                    sleep(sleep_time);
                transmit_packet(packet_wrapper);
                if (get_ack(packet_wrapper.packet->receiver_id))
                    return Result::Ok;
            }
            return Result::Fail;
        };
        auto get_ack(Id transmitter_id) const -> Result
        {
            for (auto attempts = 0; attempts < 3; attempts++)
            {
                const auto [packet, length] = receive_packet(10);
                if (length == 0)
                    continue;
                const auto is_receiver_ok = packet->receiver_id == id;
                const auto is_transmitter_ok = packet->transmitter_id == transmitter_id;
                const auto is_msg_type_ok = packet->msg_type == MsgType::Ack;
                const auto is_everything_ok = is_receiver_ok && is_transmitter_ok && is_msg_type_ok;
                if (is_everything_ok)
                    return Result::Ok;
            }
            return Result::Fail;
        };
        auto transmit_i_am_parent() const -> void
        {
            const Header packet = {
                MsgType::IAmParent,
                id,
                broadcast,
            };
            sleep(sleep_time);
            while (is_channel_busy())
                sleep(sleep_time);
            transmit(packet);
        }
        auto send_own_data(uint32_t parent_id) const -> void
        {
            data_packet->receiver_id = parent_id;
            deliver({data_packet,
                     header_size + data_length});
        }
        auto send_end_of_data(uint32_t parent_id) const -> void
        {
            static Packet packet = {
                MsgType::EndOfData,
                id,
                0};
            packet.receiver_id = parent_id;
            deliver({&packet,
                     header_size});
        }
        auto send_ack(Id receiver_id) const -> void
        {
            Header packet = {
                MsgType::Ack,
                id,
                0,
            };
            packet.receiver_id = receiver_id;
            while (is_channel_busy())
                sleep(sleep_time);
            transmit(packet);
        }
        auto receive_packet(uint32_t timeout) const -> PacketWrapper
        {
            const auto [buffer, length] = receive(timeout);
            return {reinterpret_cast<Packet *>(buffer), length};
        }
        auto transmit_packet(ConstPacketWrapper packet) const -> void
        {
            const ConstBytes bytes = {
                reinterpret_cast<const uint8_t *>(packet.packet),
                packet.length,
            };
            return transmit(bytes);
        }
    };
}

#endif
