import React from "react";
import { AuthProvider } from "../hooks/useAuth";
import Chatbot from "../components/Chatbot/Chatbot";

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
