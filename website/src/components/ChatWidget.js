// website/src/components/ChatWidget.js
import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router'; // To get current URL
import './ChatWidget.css'; // For custom styles


// Define interfaces for request/response (simplified for JS)
/**
 * @typedef {object} FrontendRequest
 * @property {string} query
 * @property {string} [source_url_constraint]
 * @property {string} [section_constraint]
 * @property {string} [selected_text_constraint]
 */

/**
 * @typedef {object} Citation
 * @property {string} source_url
 * @property {string} section
 * @property {string} raw_text_snippet
 */

/**
 * @typedef {object} FrontendResponse
 * @property {string} answer
 * @property {Array<Citation>} citations
 * @property {string} [message]
 */


// Utility function to get the nearest section heading
const getNearestSection = () => {
  if (typeof document === 'undefined') return null; // Ensure document is defined

  const headings = Array.from(document.querySelectorAll('h1, h2, h3, h4, h5, h6'));
  let nearestHeading = null;
  let minDistance = Infinity;

  const viewportCenter = window.scrollY + window.innerHeight / 2;

  for (const heading of headings) {
    const rect = heading.getBoundingClientRect();
    const headingTop = rect.top + window.scrollY;
    const distance = Math.abs(headingTop - viewportCenter);

    if (distance < minDistance) {
      minDistance = distance;
      nearestHeading = heading;
    }
  }
  return nearestHeading ? nearestHeading.textContent.trim() : null;
};

// Utility function to slugify text for URL anchors
const slugify = (text) => {
  if (!text) return '';
  return text
    .toString()
    .normalize('NFD')
    .replace(/[\u0300-\u036f]/g, '')
    .toLowerCase()
    .trim()
    .replace(/\s+/g, '-')
    .replace(/[^\w-]+/g, '')
    .replace(/--+/g, '-');
};


const CHAT_HISTORY_KEY = 'rag_chat_history';
const CHAT_BUTTON_POSITION_KEY = 'rag_chat_button_position';

function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  // Initialize messages from localStorage
  const [messages, setMessages] = useState(() => {
    if (typeof window !== 'undefined') {
      const savedHistory = localStorage.getItem(CHAT_HISTORY_KEY);
      return savedHistory ? JSON.parse(savedHistory) : [];
    }
    return [];
  });
  const [query, setQuery] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [selectionRect, setSelectionRect] = useState(null); // To position the "Ask about this" button

  const chatButtonRef = useRef(null); // Ref for the floating button

  // Draggable button state
  const [buttonPosition, setButtonPosition] = useState(() => {
    if (typeof window !== 'undefined') {
      const savedPosition = localStorage.getItem(CHAT_BUTTON_POSITION_KEY);
      if (savedPosition) {
        return JSON.parse(savedPosition);
      }
      // Default position if not saved
      return { x: window.innerWidth - 70, y: window.innerHeight - 70 };
    }
    return { x: 0, y: 0 };
  });
  const [isDragging, setIsDragging] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });

  const location = useLocation();
  const currentUrl = typeof window !== 'undefined' ? window.location.href : '';

  // Get API URL from environment variables
  const RAG_AGENT_API_URL = (typeof process !== 'undefined' && process.env?.REACT_APP_RAG_AGENT_API_URL) || 'http://localhost:8000/agent/ask';

  // Effect to capture selected text and its position
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection ? selection.toString().trim() : '';
      setSelectedText(text);

      if (text && selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionRect({
          left: rect.left + window.scrollX,
          top: rect.top + window.scrollY,
          width: rect.width,
          height: rect.height,
        });
      } else {
        setSelectionRect(null);
      }
    };

    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  // Effect for draggable button functionality
  useEffect(() => {
    const handleMouseMove = (event) => {
      if (isDragging) {
        setButtonPosition({
          x: event.clientX - dragOffset.x,
          y: event.clientY - dragOffset.y,
        });
      }
    };

    const handleMouseUp = () => {
      setIsDragging(false);
    };

    if (isDragging) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
    }

    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, dragOffset]);

  // Effect to save messages to localStorage whenever messages state changes
  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
    }
  }, [messages]);

  // Effect to save button position to localStorage whenever it changes
  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem(CHAT_BUTTON_POSITION_KEY, JSON.stringify(buttonPosition));
    }
  }, [buttonPosition]);


  const toggleChat = () => {
    setIsOpen(prev => !prev);
    // Clear selected text, query, and selection rect when chat is closed
    if (isOpen) {
      setSelectedText('');
      setSelectionRect(null);
      setQuery(''); // Clear pending input
    }
  };

  const handleQueryChange = (event) => {
    setQuery(event.target.value);
  };

  const sendQueryToBackend = async (userQuery, constraints = {}) => {
    setIsLoading(true);
    setError(null);

    const nearestSection = getNearestSection(); // Get nearest section for context

    const requestPayload = {
      query: userQuery,
      source_url_constraint: constraints.source_url_constraint || currentUrl,
      section_constraint: constraints.section_constraint || nearestSection,
      selected_text_constraint: constraints.selected_text_constraint,
    };

    try {
      const response = await fetch(RAG_AGENT_API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestPayload),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      /** @type {FrontendResponse} */
      const data = await response.json();
      setMessages(prevMessages => [
        ...prevMessages,
        { type: 'agent', text: data.answer, citations: data.citations, message: data.message }
      ]);
    } catch (err) {
      console.error('Failed to fetch from RAG agent:', err);
      setError('Failed to get response from agent. Please try again later.');
      setMessages(prevMessages => [
        ...prevMessages,
        { type: 'agent', text: 'Error: Failed to get response. Please try again.', citations: [] }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  // Helper function to process query and send message
  const processAndSendMessage = async (userQuery, constraints = {}) => {
    if (isLoading) return; // Prevent sending if already loading

    const nearestSection = getNearestSection();
    const requestPayload = {
      query: userQuery,
      source_url_constraint: constraints.source_url_constraint || currentUrl,
      section_constraint: constraints.section_constraint || nearestSection,
      selected_text_constraint: constraints.selected_text_constraint,
    };

    setMessages(prevMessages => [...prevMessages, { type: 'user', text: userQuery, sentConstraints: requestPayload }]);
    await sendQueryToBackend(userQuery, requestPayload); // Pass requestPayload to sendQueryToBackend
  };

  const handleSubmitQuery = (event) => {
    event.preventDefault();
    if (isLoading) return; // Prevent multiple simultaneous queries
    if (query.trim()) {
      processAndSendMessage(query, selectedText ? { selected_text_constraint: selectedText } : {});
      setQuery('');
      setSelectedText(''); // Clear selected text after submitting
      setSelectionRect(null);
    }
  };

  const handleAskAboutSelection = async () => {
    if (isLoading) return; // Prevent multiple simultaneous queries
    if (selectedText) {
      setIsOpen(true); // Open chat if closed
      const userQuery = `Ask about "${selectedText}"`; // Pre-populate query field

      processAndSendMessage(userQuery, { selected_text_constraint: selectedText, source_url_constraint: currentUrl, section_constraint: getNearestSection() });

      setQuery(''); // Clear query after sending
      setSelectedText(''); // Clear selected text after submitting
      setSelectionRect(null);
    }
  };

  const clearChat = () => {
    setMessages([]);
    localStorage.removeItem(CHAT_HISTORY_KEY);
  };

  return (
    <>
      {/* Floating Button */}
      <button
        ref={chatButtonRef}
        onClick={toggleChat}
        onMouseDown={e => {
          setIsDragging(true);
          setDragOffset({ x: e.clientX - chatButtonRef.current.getBoundingClientRect().left, y: e.clientY - chatButtonRef.current.getBoundingClientRect().top });
        }}
        className="chat-toggle-button" // Use CSS class for styling
        title={isOpen ? 'Close Chat' : 'Open Chat'}
        style={{
          position: 'fixed',
          left: buttonPosition.x,
          top: buttonPosition.y,
          cursor: isDragging ? 'grabbing' : 'grab',
          // Ensure it's above other content but below the panel when open
          zIndex: isOpen ? 999 : 1000,
        }}
      >
        {isOpen ? 'X' : 'Chat'}
      </button>

      {/* "Ask about this" button for selected text */}
      {selectedText && selectionRect && (
        <button
          onClick={handleAskAboutSelection}
          className="ask-about-selection-button"
          style={{
            position: 'absolute',
            left: selectionRect.left + selectionRect.width / 2 - 75, // Center above selection
            top: selectionRect.top - 40, // Above selection
            zIndex: 1001, // Above chat widget
            backgroundColor: '#007bff',
            color: 'white',
            border: 'none',
            borderRadius: '5px',
            padding: '5px 10px',
            cursor: 'pointer',
            boxShadow: '0 2px 5px rgba(0,0,0,0.2)',
          }}
        >
          Ask about this
        </button>
      )}


      {/* Chat Panel */}
      {isOpen && (
        <div className="chat-panel">
          <div className="chat-header">
            Chat with RAG Agent
            <div className="header-controls">
              <button className="chat-clear-button" onClick={clearChat} title="Clear History">🗑️</button>
              <button className="chat-close-button" onClick={toggleChat}>X</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`chat-message chat-message-${msg.type}`}>
                <span>{msg.text}</span>
                {msg.type === 'user' && msg.sentConstraints && (
                  <div className="constraints-display">
                    {msg.sentConstraints.selected_text_constraint && (
                      <p><em>Constrained by selected text:</em> "{msg.sentConstraints.selected_text_constraint}"</p>
                    )}
                    {msg.sentConstraints.source_url_constraint && (
                      <p><em>Constrained to URL:</em> <a href={msg.sentConstraints.source_url_constraint} target="_blank" rel="noopener noreferrer">{msg.sentConstraints.source_url_constraint}</a></p>
                    )}
                    {msg.sentConstraints.section_constraint && (
                      <p><em>Constrained to section:</em> "{msg.sentConstraints.section_constraint}"</p>
                    )}
                  </div>
                )}
                {msg.citations && msg.citations.length > 0 && (
                  <div className="citations-container">
                    <strong>Sources:</strong>
                    {msg.citations.map((citation, citIndex) => {
                      const citationHref = citation.section
                        ? `${citation.source_url}#${slugify(citation.section)}`
                        : citation.source_url;
                      return (
                        <div key={citIndex} className="citation-item">
                          <a href={citationHref} target="_blank" rel="noopener noreferrer">
                            {citation.section || citation.source_url}
                          </a>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className="chat-message chat-message-agent">
                <span>Agent is thinking...</span>
              </div>
            )}
            {error && (
              <div className="chat-message chat-message-error">
                <span>Error: {error}</span>
              </div>
            )}
          </div>
          <form onSubmit={handleSubmitQuery} className="chat-input-form">
            <input
              type="text"
              value={query}
              onChange={handleQueryChange}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div >
      )
      }
    </>
  );
}

export default ChatWidget;