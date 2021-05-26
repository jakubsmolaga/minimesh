#include <inttypes.h>

namespace minimesh
{

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */
    using DelayFunc = auto(uint32_t number_of_microseconds) -> void;
    using TransmitFunc = auto(const uint8_t *data, uint8_t length) -> void;
    using ReceiveFunc = auto(uint8_t *data, uint32_t timeout_ms) -> uint8_t;
    using CheckFunc = bool();
    using Id = uint32_t;
    using Byte = uint8_t;
    using Length = uint8_t;

    enum MsgType : uint32_t
    {
        IAmParent,
        IAmChild,
        Data,
        EndOfData,
        Ack,
    };

    enum Status
    {
        Fail,
        Ok,
    };

    struct Packet
    {
        MsgType msg_type;           // Type of message
        Id transmitter_id;          // Id of transmitting device
        Id receiver_id;             // Id of intended receiver (0 means broadcast)
        Byte data[max_data_length]; // Custom data
    };

    struct PacketWrapper
    {
        Packet packet; // A packet
        Length length; // Length of the whole packet (including headers). 0 means no packet
    };

    constexpr Length max_packet_size = 255;
    constexpr Length header_size = sizeof(MsgType) + sizeof(Id) + sizeof(Id);
    constexpr Length max_data_length = max_packet_size - header_size;

    /* -------------------------------------------------------------------------- */
    /*                               Implementation                               */
    /* -------------------------------------------------------------------------- */

    struct Handle
    {
        // Main procedure
        void run(const uint8_t data[], const uint8_t data_length)
        {
            // Find parent
            const auto parent_id = find_parent();

            // Count children
            const auto amount_of_children = count_children();

            // Proxy data from children to parent
            proxy_children(amount_of_children, parent_id);

            // Send own data to parent
            deliver_data(data, data_length, parent_id);

            // End data stream
            deliver(MsgType::EndOfData, parent_id, 3);
        };

        // Members
        const DelayFunc *delay_us;     // Should make device go to sleep for a given number of microseconds
        const TransmitFunc *transmit;  // Transmit bytes over a medium
        const ReceiveFunc *receive;    // Should return number of bytes received
        const CheckFunc *is_line_busy; // Check if there is anyone else transmitting data at the moment
        const Id id;                   // Unique identifier of this device

    private:
        // find_parent waits for an IAmParent message, responds to it and returns parent id
        Id find_parent()
        {
            const auto [packet, length] = receive_packet(0);
            const auto parent_id = packet.transmitter_id;
            send_ack(parent_id);
            return parent_id;
        };

        auto deliver_data(const Byte data[], Length data_length, Id parent_id) -> void
        {
            const auto [packet, length] = build_data_packet(data, data_length, parent_id);
            deliver(packet, length, 3);
        };

        auto build_data_packet(const Byte data[], Length data_length, Id receiver_id) -> PacketWrapper
        {
            auto packet = (Packet){MsgType::Data, id, receiver_id, {}};
            for (auto i = 0; i < data_length; i++)
                packet.data[i] = data[i];
            const Length packet_length = data_length + header_size;
            return {packet, packet_length};
        };

        // Count all direct children
        // TODO: keep track of child ids to avoid counting duplicates
        auto count_children() -> uint32_t
        {
            uint32_t count = 0;
            while (true)
            {
                // Try to discover a child
                auto result = find_child();

                // Return if there is no more children
                if (result == Status::Fail)
                    return count;

                // Add child to the list
                count++;
            }
        };

        auto proxy_children(const uint8_t amount, const Id parent_id) -> void
        {
            if (amount == 0)
                return;
            auto [packet, length] = get_child_data();
            if (length == 0)
                return;
            if (packet.msg_type == MsgType::EndOfData)
                return proxy_children(amount - 1, parent_id);
            packet.receiver_id = parent_id;
            deliver_immediately(packet, length, 5);
            return proxy_children(amount, parent_id);
        };

        // Get a message with certain type
        // TODO: return optional Id
        auto find_child() -> Status
        {
            const auto validator = [&](Packet packet)
            {
                return (packet.receiver_id == id && packet.msg_type == MsgType::IAmChild);
            };
            const auto [packet, length] = get_with_validator(validator, 10);
            if (length == 0)
                return Status::Fail;
            send_ack(packet.transmitter_id);
            return Status::Ok;
        };

        auto get_child_data() -> PacketWrapper
        {
            const auto validator = [&](Packet p)
            {
                const auto is_type_correct = (p.msg_type == MsgType::Data || p.msg_type == MsgType::EndOfData);
                const auto is_address_correct = (p.receiver_id == id);
                return (is_address_correct && is_type_correct);
            };
            const auto [packet, length] = get_with_validator(validator, 10);
            if (length == 0)
                return {{}, 0};
            send_ack(packet.transmitter_id);
            return {packet, length};
        };

        auto get_ack(Id transmitter_id) -> Status
        {
            const auto validator = [&](Packet packet)
            {
                return (packet.msg_type == MsgType::Ack && packet.receiver_id == id && packet.transmitter_id == transmitter_id);
            };
            const auto [_, length] = get_with_validator(validator, 3);
            return length == 0 ? Status::Fail : Status::Ok;
        };

        template <typename Validator>
        auto get_with_validator(Validator is_valid, uint8_t max_attempts) -> PacketWrapper
        {
            // Fail if there are no more attempts
            if (max_attempts == 0)
                return {{}, 0};

            // Wait for a packet
            const auto [packet, length] = receive_packet(100);

            // Handle timeout
            if (length == 0)
                return {{}, 0};

            // Handle correct packet
            if (is_valid(packet))
                return {packet, length};

            // Handle wrong packet
            return get_with_validator(is_valid, max_attempts - 1);
        };

        // Send ack to some device (checks if transmission line is free)
        auto send_ack(Id receiver_id) -> void
        {
            // Build the packet
            Packet packet = {
                MsgType::Ack,
                id,
                receiver_id,
                {},
            };

            // Find time for transmission
            find_immediate_window();

            // Transmit the packet
            transmit(reinterpret_cast<Byte *>(&packet), header_size);
        };

        auto deliver_immediately(Packet packet, Length length, uint8_t max_attempts) -> Status
        {
            if (max_attempts == 0)
                return Status::Fail;
            find_immediate_window();
            transmit(reinterpret_cast<const Byte *>(&packet), length);
            if (get_ack(packet.receiver_id))
                return Status::Ok;
            return deliver_immediately(packet, length, max_attempts - 1);
        };

        auto deliver(Packet packet, Length length, uint8_t max_attempts) -> Status
        {
            if (max_attempts == 0)
                return Status::Fail;
            find_window();
            transmit(reinterpret_cast<const Byte *>(&packet), length);
            if (get_ack(packet.receiver_id))
                return Status::Ok;
            return deliver(packet, length, max_attempts - 1);
        };

        auto deliver(MsgType msg_type, Id receiver_id, uint8_t max_attempts) -> Status
        {
            if (max_attempts == 0)
                return Status::Fail;
            find_window();
            const auto packet = (Packet){msg_type, id, receiver_id, {}};
            transmit(reinterpret_cast<const Byte *>(&packet), header_size);
            if (get_ack(packet.receiver_id))
                return Status::Ok;
            return deliver(msg_type, receiver_id, max_attempts - 1);
        };

        auto sleep() -> void
        {
            delay_us(id % 9000 + 1000);
        };

        // Sleeps until transmission line is free (might not sleep at all)
        auto find_immediate_window() -> void
        {
            while (is_line_busy())
                sleep();
        };

        // Sleeps until transmission line is free (will always sleep at least once)
        auto find_window() -> void
        {
            sleep();
            find_immediate_window();
        };

        auto receive_packet(uint32_t timeout) -> PacketWrapper
        {
            Packet packet;
            auto length = receive(reinterpret_cast<Byte *>(&packet), timeout);
            return {packet, length};
        };
    };
}
