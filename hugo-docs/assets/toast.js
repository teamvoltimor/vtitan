(function() {
  'use strict';

  // Configuration constants
  const TOAST_DURATION = 3500;
  const TOAST_ANIMATION_DURATION = 300;

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

  // Create toast element
  function createToast(message, type) {
    const toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.setAttribute('role', 'status');
    toast.setAttribute('aria-live', 'polite');
    toast.innerHTML = '<span>' + message + '</span><button class="toast-close" aria-label="Close">&times;</button>';
    return toast;
  }

  // Show toast
  window.showToast = function(message, type, duration) {
    type = type || 'info';
    duration = duration || TOAST_DURATION;

    const container = ensureContainer();
    const toast = createToast(message, type);

    toast.querySelector('.toast-close').onclick = function() {
      dismissToast(toast);
    };

    container.appendChild(toast);

    setTimeout(function() {
      toast.classList.add('toast-show');
    }, 10);

    setTimeout(function() {
      dismissToast(toast);
    }, duration);
  };

  // Dismiss toast with animation
  function dismissToast(toast) {
    toast.classList.remove('toast-show');
    setTimeout(function() {
      if (toast.parentNode) {
        toast.parentNode.removeChild(toast);
      }
    }, TOAST_ANIMATION_DURATION);
  }
})();
