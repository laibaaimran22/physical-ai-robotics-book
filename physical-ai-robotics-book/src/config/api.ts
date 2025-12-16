// Configuration for API endpoints
// For Docusaurus, we'll use the custom field from the config
// This will be available as a global variable during build time
const getBackendUrl = () => {
  // Try to get the backend URL from the Docusaurus custom fields
  // This is available through the global docusaurus object
  if (typeof window !== 'undefined' && (window as any).__docusaurus) {
    const docusaurus = (window as any).__docusaurus;
    if (docusaurus.config && docusaurus.config.customFields && docusaurus.config.customFields.backendUrl) {
      return docusaurus.config.customFields.backendUrl;
    }
  }

  // Fallback to Hugging Face Space URL
  return 'https://laibaaaimran-backend-ai-book.hf.space';
};

const API_CONFIG = {
  BACKEND_URL: getBackendUrl(),
};

export default API_CONFIG;