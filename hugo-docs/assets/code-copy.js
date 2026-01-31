(function() {
  'use strict';

  // SVG icons
  const COPY_ICON = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path></svg>';
  const CHECK_ICON = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="20 6 9 17 4 12"></polyline></svg>';

  // Find all code blocks
  function addCopyButtons() {
    const codeBlocks = document.querySelectorAll('pre:not(.mermaid)');

    codeBlocks.forEach((pre) => {
      // Skip if button already exists
      if (pre.querySelector('.code-copy-button')) return;

      // Create copy button
      const button = document.createElement('button');
      button.className = 'code-copy-button';
      button.type = 'button';
      button.setAttribute('aria-label', 'Copy code to clipboard');
      button.innerHTML = COPY_ICON + '<span>Copy</span>';

      // Add click handler
      button.addEventListener('click', async () => {
        await copyCode(pre, button);
      });

      // Add button to pre element
      pre.appendChild(button);
    });
  }

  // Copy code to clipboard
  async function copyCode(pre, button) {
    // Get code element
    const code = pre.querySelector('code');
    if (!code) return;

    let text = '';

    // Check if using table-based line numbers (Hugo lineNumbersInTable)
    const lntable = code.querySelector('.lntable');
    if (lntable) {
      // Get only the code column (second td), not the line numbers column
      const codeCell = lntable.querySelector('.lntd:last-child');
      if (codeCell) {
        text = codeCell.textContent || codeCell.innerText;
      }
    } else {
      // Get text content (strip HTML)
      text = code.textContent || code.innerText;

      // Remove inline line numbers if present (when not using table)
      text = text.replace(/^\s*\d+\s+/gm, '');
    }

    try {
      // Try using the Clipboard API
      if (navigator.clipboard && window.isSecureContext) {
        await navigator.clipboard.writeText(text);
      } else {
        // Fallback for older browsers
        const textArea = document.createElement('textarea');
        textArea.value = text;
        textArea.style.position = 'fixed';
        textArea.style.left = '-999999px';
        textArea.style.top = '-999999px';
        document.body.appendChild(textArea);
        textArea.focus();
        textArea.select();

        try {
          document.execCommand('copy');
        } catch (err) {
          console.error('Failed to copy:', err);
          return;
        } finally {
          document.body.removeChild(textArea);
        }
      }

      // Show success state
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
    }, 2000);
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
      let shouldUpdate = false;
      mutations.forEach((mutation) => {
        if (mutation.addedNodes.length > 0) {
          mutation.addedNodes.forEach((node) => {
            if (node.nodeName === 'PRE' || (node.querySelector && node.querySelector('pre'))) {
              shouldUpdate = true;
            }
          });
        }
      });
      if (shouldUpdate) {
        addCopyButtons();
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }
})();
