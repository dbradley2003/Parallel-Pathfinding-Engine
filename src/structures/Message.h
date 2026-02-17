#ifndef MESSAGE_H
#define MESSAGE_H

class Message {
public:
  enum class MessageType {
    RejectRequest,
    RequestWork,
    Success,
    Empty,
  };

  explicit Message(MessageType const &type) : type_(type) {}
  [[nodiscard]] MessageType getType() const { return this->type_; }
  void setType(const MessageType type) {
    this->type_ = type;
  }
  MessageType type_;
};

#endif
