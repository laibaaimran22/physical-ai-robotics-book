import { betterAuth } from "better-auth";
import { nextCookies } from "better-auth/next-js";

// Configure Better Auth with FileSystem adapter
export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "file:./book_platform.db",
  },
  // Define additional user fields for background information
  user: {
    fields: {
      softwareLevel: {
        type: "string",
        required: false,
      },
      hardwareLevel: {
        type: "string",
        required: false,
      },
      preferredLanguages: {
        type: "string", // JSON string for array of languages
        required: false,
      },
      goals: {
        type: "string",
        required: false,
      },
    },
  },
  // Use cookies for session management
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60, // 24 hours
  },
  // Configure social providers if needed (optional)
  socialProviders: {
    // Add social providers here if needed
  },
  // API configuration
  api: {
    defaultRedirect: "/dashboard", // Default redirect after login
  },
  // Email configuration (optional)
  email: {
    // Configure email verification if needed
  },
  // Account configuration
  account: {
    accountLinking: {
      enabled: true,
    },
  },
});

// Export the cookies configuration for Next.js
export const cookies = nextCookies();