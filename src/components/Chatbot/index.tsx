import React, { useState, useEffect, useRef } from 'react';
import { chatAPI, ChatResponse, SourceReference } from '../../services/chatApi';
import { captureTextSelection, hasTextSelection, SelectionContext } from '../../utils/textSelection';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceReference[];
}

export default function ChatBot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [selectionContext, setSelectionContext] = useState<SelectionContext | null>(null);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Check for text selection when opening chat
  useEffect(() => {
    if (isOpen && hasTextSelection()) {
      const context = captureTextSelection();
      if (context) {
        setSelectionContext(context);
      }
    }
  }, [isOpen]);

  // Focus input when opening
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input.trim(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      let response: ChatResponse;

      // Create conversation if needed
      if (!conversationId) {
        const conv = await chatAPI.createConversation();
        setConversationId(conv.conversation_id);
      }

      // Send query with or without selection context
      if (selectionContext) {
        response = await chatAPI.queryWithContext({
          query: userMessage.content,
          selected_text: selectionContext.text,
          selection_metadata: {
            chapter_title: selectionContext.chapterTitle,
            section_title: selectionContext.sectionTitle,
            url: selectionContext.url,
          },
          conversation_id: conversationId || undefined,
        });
        setSelectionContext(null); // Clear after use
      } else {
        response = await chatAPI.query({
          query: userMessage.content,
          conversation_id: conversationId || undefined,
        });
      }

      const assistantMessage: Message = {
        id: response.message_id,
        role: 'assistant',
        content: response.answer,
        sources: response.sources,
      };

      setMessages(prev => [...prev, assistantMessage]);
      setConversationId(response.conversation_id);
    } catch (err) {
      console.error('Chat error:', err);
      setError('Failed to get response. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && hasTextSelection()) {
      const context = captureTextSelection();
      if (context) {
        setSelectionContext(context);
      }
    }
  };

  if (!isOpen) {
    return (
      <div className={styles.chatbotContainer}>
        <button
          className={styles.chatbotFab}
          onClick={toggleChat}
          aria-label="Open AI Assistant"
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.chatbotPanel}>
        <div className={styles.chatbotHeader}>
          <h3>AI Teaching Assistant</h3>
          <button
            className={styles.chatbotClose}
            onClick={toggleChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        <div className={styles.chatbotMessages}>
          {messages.length === 0 && (
            <div className={`${styles.chatbotMessage} ${styles.assistant}`}>
              <div className={`${styles.messageBubble} ${styles.assistant}`}>
                Hi! I'm your AI teaching assistant for the Physical AI textbook.
                Ask me anything about robotics, ROS 2, simulations, or any chapter content!
              </div>
            </div>
          )}

          {messages.map(message => (
            <div
              key={message.id}
              className={`${styles.chatbotMessage} ${styles[message.role]}`}
            >
              <div className={`${styles.messageBubble} ${styles[message.role]}`}>
                {message.content}
              </div>
            </div>
          ))}

          {isLoading && (
            <div className={`${styles.chatbotMessage} ${styles.assistant}`}>
              <div className={styles.loadingIndicator}>
                <div className={styles.loadingDot}></div>
                <div className={styles.loadingDot}></div>
                <div className={styles.loadingDot}></div>
              </div>
            </div>
          )}

          {error && (
            <div className={styles.errorMessage}>
              {error}
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <div className={styles.chatbotInputContainer}>
          {selectionContext && (
            <div className={styles.selectionContext}>
              <div className={styles.selectionText}>
                Selected: "{selectionContext.text.substring(0, 100)}..."
              </div>
              <button
                className={styles.clearSelection}
                onClick={() => setSelectionContext(null)}
                aria-label="Clear selection"
              >
                ×
              </button>
            </div>
          )}

          <div className={styles.chatbotInputWrapper}>
            <textarea
              ref={inputRef}
              className={styles.chatbotInput}
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about Physical AI..."
              rows={1}
              disabled={isLoading}
            />
            <button
              className={styles.chatbotSend}
              onClick={handleSend}
              disabled={!input.trim() || isLoading}
            >
              Send
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
