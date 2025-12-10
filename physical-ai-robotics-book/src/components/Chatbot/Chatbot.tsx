import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';
import ChatbotIcon from './ChatbotIcon';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

const Chatbot: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      content: 'Hello! I\'m your AI assistant for the Physical AI and Humanoid Robotics Book. Ask me anything about the content!',
      role: 'assistant',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';
    if (text.length > 0) {
      setSelectedText(text);
      return text;
    }
    return '';
  };

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle sending a message with optional context
  const handleSubmit = async (e: React.FormEvent, contextText = '') => {
    e.preventDefault();
    if ((!inputValue.trim() && !contextText) || isLoading) return;

    const userMessage = contextText || inputValue;
    if (!contextText) {
      setInputValue('');
    }

    // Add user message
    const userMessageObj: Message = {
      id: Date.now().toString(),
      content: userMessage,
      role: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessageObj]);
    if (!contextText) {
      setInputValue('');
    }
    setIsLoading(true);

    try {
      // Determine which API endpoint to use based on context
      const endpoint = contextText ? '/api/v1/rag/query-selected' : '/api/v1/rag/query';

      const requestBody = contextText
        ? {
            query: inputValue || `About: ${contextText.substring(0, 100)}...`,
            selected_text: contextText
          }
        : { query: userMessage };

      // Call the backend RAG API
      const response = await fetch(`http://127.0.0.1:8000${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: data.response || data.answer || data.text || JSON.stringify(data),
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error fetching response:', error);

      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        role: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to handle text selection query
  const handleSelectedTextQuery = () => {
    const selected = getSelectedText();
    if (!selected) {
      alert('Please select some text on the page first.');
      return;
    }

    // Pre-fill the input with a query about the selected text
    setInputValue(`Explain this concept: "${selected.substring(0, 50)}${selected.length > 50 ? '...' : ''}"`);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Function to clear chat
  const handleClearChat = () => {
    setMessages([]);
    setSelectedText('');
  };

  return (
    <>
      {/* Chatbot toggle button */}
      <button
        className={clsx(styles.chatbotToggle, {
          [styles.chatbotToggleOpen]: isOpen
        })}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <ChatbotIcon isOpen={isOpen} />
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div className={styles.chatbotContainer}>
          <div className={styles.chatbotHeader}>
            <h3>AI Assistant</h3>
            <div className={styles.headerActions}>
              <button
                onClick={handleSelectedTextQuery}
                className={styles.selectTextButton}
                title="Ask about selected text"
              >
                🔍 Ask about Selection
              </button>
              <button
                onClick={handleClearChat}
                className={styles.clearChatButton}
                title="Clear chat"
              >
                🗑️ Clear
              </button>
            </div>
          </div>

          <div className={styles.chatbotMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
                <p>You can:</p>
                <ul>
                  <li>Ask questions about the book content</li>
                  <li>Select text on the page and click "Ask about Selection"</li>
                  <li>Ask for explanations of specific concepts</li>
                </ul>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={clsx(
                    styles.message,
                    message.role === 'user' ? styles.userMessage : styles.assistantMessage
                  )}
                >
                  <div className={styles.messageContent}>
                    <strong>{message.role === 'user' ? 'You:' : 'AI Assistant:'}</strong>
                    <p>{message.content}</p>
                  </div>
                  <div className={styles.messageTimestamp}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={clsx(styles.message, styles.assistantMessage)}>
                <div className={styles.messageContent}>
                  <strong>AI Assistant:</strong>
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

          <form onSubmit={handleSubmit} className={styles.chatbotInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about Physical AI and Robotics..."
              className={styles.chatbotInput}
              disabled={isLoading}
              aria-label="Type your message"
            />
            <button
              type="submit"
              className={styles.chatbotSubmitButton}
              disabled={isLoading || (!inputValue.trim() && !selectedText)}
              aria-label="Send message"
            >
              {isLoading ? 'Sending...' : '→'}
            </button>
          </form>

          {selectedText && (
            <div className={styles.selectedTextPreview}>
              <p><strong>Selected Text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
              <button
                onClick={(e) => {
                  e.preventDefault();
                  handleSubmit(e, selectedText);
                }}
                disabled={isLoading}
              >
                Ask about this text
              </button>
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default Chatbot;