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

	unsigned int senderIndex;
	char pad2[4];
	MessageType type;
	char pad[4];
};

#endif

