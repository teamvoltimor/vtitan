(function() {
  'use strict';

  // Configuration constants
  const TOAST_DURATION = 3500;

  let toastContainer = null;

  // Create toast container
  function ensureContainer() {
    if (toastContainer) return toastContainer;

    toastContainer = document.createElement('div');
    toastContainer.id = 'toast-container';
    toastContainer.setAttribute('aria-live', 'polite');
    toastContainer.setAttribute('aria-atomic', 'true');
    document.body.appendChild(toastContainer);
    return toastContainer;
  }

  // Show toast
  window.showToast = function(message, type, duration) {
    type = type || 'info';
    duration = duration || TOAST_DURATION;

    const container = ensureContainer();

    const toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.setAttribute('role', 'status');
    toast.setAttribute('aria-live', 'polite');
    toast.textContent = message;

    container.appendChild(toast);

    setTimeout(function() {
      if (toast.parentNode) {
        toast.parentNode.removeChild(toast);
      }
    }, duration);
  };
})();
