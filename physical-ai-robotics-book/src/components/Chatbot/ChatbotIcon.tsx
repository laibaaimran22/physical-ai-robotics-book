import React from 'react';
import clsx from 'clsx';
import styles from './ChatbotIcon.module.css';

interface ChatbotIconProps {
  isOpen?: boolean;
  className?: string;
}

const ChatbotIcon: React.FC<ChatbotIconProps> = ({ isOpen = false, className }) => {
  return (
    <div className={clsx(styles.chatbotIconContainer, className)}>
      <svg
        className={clsx(styles.chatbotIcon, { [styles.open]: isOpen })}
        viewBox="0 0 24 24"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Robot head with AI elements */}
        <path
          d="M12 2C8.13 2 5 5.13 5 9C5 11.08 5.85 12.95 7.24 14.28C7.09 14.89 7 15.53 7 16.18C7 16.83 7.09 17.47 7.24 18.08C5.85 19.41 5 21.28 5 23.36C5 23.71 5.29 24 5.64 24H18.36C18.71 24 19 23.71 19 23.36C19 21.28 18.15 19.41 16.76 18.08C16.91 17.47 17 16.83 17 16.18C17 15.53 16.91 14.89 16.76 14.28C18.15 12.95 19 11.08 19 9C19 5.13 15.87 2 12 2Z"
          fill="currentColor"
        />

        {/* AI circuit pattern eyes */}
        <circle cx="9" cy="10" r="1" fill="white" />
        <circle cx="15" cy="10" r="1" fill="white" />

        {/* AI circuit pattern mouth */}
        <path
          d="M10 14C10 14.5523 10.4477 15 11 15H13C13.5523 15 14 14.5523 14 14"
          stroke="white"
          strokeWidth="1"
          strokeLinecap="round"
        />

        {/* Circuit board pattern on head */}
        <path
          d="M11 6H13"
          stroke="white"
          strokeWidth="0.5"
          strokeLinecap="round"
        />
        <path
          d="M10 7H14"
          stroke="white"
          strokeWidth="0.5"
          strokeLinecap="round"
        />
        <path
          d="M11 8H13"
          stroke="white"
          strokeWidth="0.5"
          strokeLinecap="round"
        />
      </svg>
    </div>
  );
};

export default ChatbotIcon;