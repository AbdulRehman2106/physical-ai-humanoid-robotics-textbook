import React from 'react';
import { MessageList } from './MessageList';
import { InputBox } from './InputBox';
import { chatApi, Message } from '../services/chatApi';
import './ChatInterface.css';

export const ChatInterface: React.FC = () => {
  const [conversationId, setConversationId] = React.useState<string | null>(null);
  const [messages, setMessages] = React.useState<Message[]>([]);
  const [isLoading, setIsLoading] = React.useState(false);
  const [error, setError] = React.useState<string | null>(null);

  // Create conversation on mount
  React.useEffect(() => {
    const initConversation = async () => {
      try {
        const conversation = await chatApi.createConversation();
        setConversationId(conversation.id);
      } catch (err: any) {
        setError('Failed to initialize conversation. Please refresh the page.');
        console.error('Failed to create conversation:', err);
      }
    };

    initConversation();
  }, []);

  const handleSendMessage = async (content: string) => {
    if (!conversationId) {
      setError('No active conversation. Please refresh the page.');
      return;
    }

    // Add user message to UI immediately
    const userMessage: Message = {
      id: `temp-${Date.now()}`,
      conversationId,
      role: 'user',
      content,
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Send message to API
      const assistantMessage = await chatApi.sendMessage(conversationId, content);

      // Add assistant response to UI
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err: any) {
      const errorMessage = err.response?.data?.message || 'Failed to send message. Please try again.';
      setError(errorMessage);
      console.error('Failed to send message:', err);

      // Remove the temporary user message on error
      setMessages((prev) => prev.filter((msg) => msg.id !== userMessage.id));
    } finally {
      setIsLoading(false);
    }
  };

  const handleNewConversation = async () => {
    try {
      const conversation = await chatApi.createConversation();
      setConversationId(conversation.id);
      setMessages([]);
      setError(null);
    } catch (err: any) {
      setError('Failed to create new conversation. Please try again.');
      console.error('Failed to create conversation:', err);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <div className="header-content">
          <h1>🤖 AI Chatbot Assistant</h1>
          <p>Powered by Cohere • Professional, context-aware assistance</p>
        </div>
        <button className="new-chat-button" onClick={handleNewConversation}>
          New Chat
        </button>
      </div>

      {error && (
        <div className="error-banner">
          <span>⚠️ {error}</span>
          <button onClick={() => setError(null)}>✕</button>
        </div>
      )}

      <MessageList messages={messages} isLoading={isLoading} />

      <InputBox onSendMessage={handleSendMessage} disabled={isLoading || !conversationId} />
    </div>
  );
};
