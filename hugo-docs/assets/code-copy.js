(function() {
  'use strict';

  // Configuration constants
  const COPY_FEEDBACK_DURATION = 2000;
  const FALLBACK_TEXT_AREA_HIDDEN_POSITION = '-999999px';
  const CODE_BLOCK_SELECTOR = 'pre:not(.mermaid)';
  const COPY_BUTTON_CLASS = 'code-copy-button';

  // SVG icons
  const COPY_ICON = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path></svg>';
  const CHECK_ICON = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="20 6 9 17 4 12"></polyline></svg>';

  // Check if button already exists on element
  function buttonExists(pre) {
    return pre.querySelector('.' + COPY_BUTTON_CLASS);
  }

  // Create copy button element
  function createCopyButton() {
    const button = document.createElement('button');
    button.className = COPY_BUTTON_CLASS;
    button.type = 'button';
    button.setAttribute('aria-label', 'Copy code to clipboard');
    button.innerHTML = COPY_ICON + '<span>Copy</span>';
    return button;
  }

  // Attach button to code block
  function attachButton(pre, button) {
    pre.appendChild(button);
  }

  // Handle click event on button
  function handleClick(pre, button) {
    button.addEventListener('click', async () => {
      await copyCode(pre, button);
    });
  }

  // Find all code blocks
  function addCopyButtons() {
    const codeBlocks = document.querySelectorAll(CODE_BLOCK_SELECTOR);

    codeBlocks.forEach((pre) => {
      if (buttonExists(pre)) return;

      const button = createCopyButton();
      handleClick(pre, button);
      attachButton(pre, button);
    });
  }

  // Extract text from code element
  function extractCodeText(code) {
    const text = code.textContent || code.innerText;
    return text.replace(/^\s*\d+\s+/gm, '');
  }

  // Write text using Clipboard API
  async function copyToClipboard(text) {
    if (navigator.clipboard && window.isSecureContext) {
      await navigator.clipboard.writeText(text);
      return true;
    }
    return copyToClipboardFallback(text);
  }

  // Fallback copy for older browsers
  function copyToClipboardFallback(text) {
    const textArea = document.createElement('textarea');
    textArea.value = text;
    textArea.style.position = 'fixed';
    textArea.style.left = FALLBACK_TEXT_AREA_HIDDEN_POSITION;
    textArea.style.top = FALLBACK_TEXT_AREA_HIDDEN_POSITION;
    document.body.appendChild(textArea);
    textArea.focus();
    textArea.select();

    const success = document.execCommand('copy');
    document.body.removeChild(textArea);
    return success;
  }

  // Copy code to clipboard
  async function copyCode(pre, button) {
    const code = pre.querySelector('code');
    if (!code) return;

    const text = extractCodeText(code);

    try {
      await copyToClipboard(text);
      showCopied(button);
    } catch (err) {
      console.error('Failed to copy code:', err);
    }
  }

  // Show copied state
  function showCopied(button) {
    const originalHTML = button.innerHTML;
    button.innerHTML = CHECK_ICON + '<span>Copied!</span>';
    button.classList.add('copied');

    setTimeout(() => {
      button.innerHTML = originalHTML;
      button.classList.remove('copied');
    }, COPY_FEEDBACK_DURATION);
  }

  // Initialize
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', addCopyButtons);
  } else {
    addCopyButtons();
  }

  // Re-add buttons if content is dynamically loaded
  if (typeof MutationObserver !== 'undefined') {
    const observer = new MutationObserver((mutations) => {
      const hasNewCodeBlocks = mutations.some((mutation) => {
        return Array.from(mutation.addedNodes).some((node) => {
          return node.nodeName === 'PRE' || (node.querySelector && node.querySelector('pre'));
        });
      });

      if (hasNewCodeBlocks) {
        addCopyButtons();
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }
})();
