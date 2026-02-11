/**
 * Full-Page Chat Interface
 * Dedicated page for AI chatbot interaction
 */

import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './chat.module.css';

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
}

const API_BASE_URL = 'http://localhost:3001/api';

export default function ChatPage() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  useEffect(() => {
    initializeConversation();
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const initializeConversation = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/conversations`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
      });

      if (!response.ok) throw new Error('Failed to create conversation');

      const data = await response.json();
      setConversationId(data.id);
    } catch (err) {
      setError('Failed to initialize chat. Please refresh the page.');
      console.error('Conversation init error:', err);
    }
  };

  const handleSend = async () => {
    if (!input.trim() || isLoading || !conversationId) return;

    const userMessage = input.trim();
    setInput('');
    setError(null);

    const tempUserMessage: ChatMessage = {
      id: `temp-${Date.now()}`,
      role: 'user',
      content: userMessage,
      timestamp: new Date().toISOString(),
    };
    setMessages(prev => [...prev, tempUserMessage]);

    setIsLoading(true);
    try {
      const response = await fetch(
        `${API_BASE_URL}/chat/conversations/${conversationId}/messages`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ content: userMessage }),
        }
      );

      if (!response.ok) throw new Error('Failed to get response');

      const assistantMessage = await response.json();
      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError('Failed to get response. Please try again.');
      const errorMessage: ChatMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: "I'm sorry, I encountered an error. Please try again.",
        timestamp: new Date().toISOString(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleNewChat = async () => {
    setMessages([]);
    setError(null);

    if (conversationId) {
      try {
        await fetch(`${API_BASE_URL}/chat/conversations/${conversationId}`, {
          method: 'DELETE',
        });
      } catch (err) {
        console.error('Failed to delete conversation:', err);
      }
    }

    setConversationId(null);
    await initializeConversation();
  };

  return (
    <Layout
      title="AI Chat Assistant"
      description="Chat with AI about Physical AI and Robotics"
    >
      <div className={styles.chatPageContainer}>
        <div className={styles.chatHeader}>
          <div className={styles.headerContent}>
            <h1>🤖 AI Chat Assistant</h1>
            <p>Ask me anything about Physical AI, ROS 2, Robotics, and more!</p>
          </div>
          <button className={styles.newChatButton} onClick={handleNewChat}>
            New Chat
          </button>
        </div>

        {error && (
          <div className={styles.errorBanner}>
            <span>⚠️ {error}</span>
            <button onClick={() => setError(null)}>✕</button>
          </div>
        )}

        <div className={styles.chatContent}>
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeScreen}>
                <div className={styles.welcomeIcon}>💬</div>
                <h2>Welcome to AI Chat!</h2>
                <p>Start a conversation by typing a message below.</p>
                <div className={styles.suggestions}>
                  <h3>Try asking:</h3>
                  <button onClick={() => setInput("What is Physical AI?")}>
                    What is Physical AI?
                  </button>
                  <button onClick={() => setInput("Explain ROS 2 nodes and topics")}>
                    Explain ROS 2 nodes and topics
                  </button>
                  <button onClick={() => setInput("How does sim-to-real transfer work?")}>
                    How does sim-to-real transfer work?
                  </button>
                  <button onClick={() => setInput("What are VLA models?")}>
                    What are VLA models?
                  </button>
                </div>
              </div>
            )}

            {messages.map((msg) => (
              <div
                key={msg.id}
                className={`${styles.message} ${
                  msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageHeader}>
                  <span className={styles.messageRole}>
                    {msg.role === 'user' ? '👤 You' : '🤖 Assistant'}
                  </span>
                  <span className={styles.messageTime}>
                    {new Date(msg.timestamp).toLocaleTimeString([], {
                      hour: '2-digit',
                      minute: '2-digit',
                    })}
                  </span>
                </div>
                <div className={styles.messageContent}>{msg.content}</div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.messageHeader}>
                  <span className={styles.messageRole}>🤖 Assistant</span>
                </div>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <div className={styles.inputWrapper}>
              <textarea
                ref={textareaRef}
                className={styles.input}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Type your message... (Shift+Enter for new line)"
                disabled={isLoading || !conversationId}
                rows={3}
              />
              <button
                className={styles.sendButton}
                onClick={handleSend}
                disabled={!input.trim() || isLoading || !conversationId}
              >
                {isLoading ? '⏳' : 'Send ➤'}
              </button>
            </div>
            <div className={styles.inputHint}>
              Press Enter to send • Shift+Enter for new line
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
