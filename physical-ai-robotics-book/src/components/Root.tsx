import React from 'react';
import Chatbot from './Chatbot/Chatbot';

// This Root component will be used to wrap the entire app
const Root = ({ children }) => {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
};

export default Root;