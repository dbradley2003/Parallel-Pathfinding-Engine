#ifndef MESSAGE_H
#define MESSAGE_H

class Message
{
public:
	enum class MessageType
	{
		RejectRequest,
		RequestWork,
		Success,
		Uninitialized,
	};
    Message(MessageType const& type) : type_(type) {}
    MessageType getType()
    {
        return this->type_;
    }
	MessageType type_;
};

#endif

