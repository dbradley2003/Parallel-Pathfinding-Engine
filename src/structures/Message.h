#ifndef MESSAGE_H
#define MESSAGE_H

#include <functional>

#include "Frontier.h"

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


	std::function<void(Frontier& other)> callback;
	unsigned int senderIndex;
	char pad2[4];
	MessageType type;
	char pad[4];
};

#endif

