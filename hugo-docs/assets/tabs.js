(function() {
  'use strict';

  // Initialize all tabs on the page
  function initTabs() {
    const tabContainers = document.querySelectorAll('[data-tabs]');

    tabContainers.forEach((container, containerIndex) => {
      const tabPanes = container.querySelectorAll('.tab-pane');
      const tabsHeader = container.querySelector('.tabs-header');

      if (!tabsHeader || tabPanes.length === 0) return;

      // Create tab buttons
      const buttons = [];
      tabPanes.forEach((pane, index) => {
        const tabName = pane.getAttribute('data-tab-name') || `Tab ${index + 1}`;
        const button = createTabButton(tabName, index, containerIndex);
        buttons.push(button);
        tabsHeader.appendChild(button);
      });

      // Activate first tab by default or from URL hash
      const hash = window.location.hash.slice(1);
      let initialTab = 0;

      if (hash) {
        const hashIndex = Array.from(tabPanes).findIndex(pane => pane.id === hash);
        if (hashIndex !== -1) {
          initialTab = hashIndex;
        }
      }

      activateTab(container, initialTab);

      // Add click handlers
      buttons.forEach((button, index) => {
        button.addEventListener('click', () => {
          activateTab(container, index);
          // Update URL hash if tab has an ID
          const pane = tabPanes[index];
          if (pane.id) {
            history.pushState(null, null, `#${pane.id}`);
          }
        });
      });

      // Keyboard navigation
      tabsHeader.addEventListener('keydown', (e) => {
        handleKeyboard(e, buttons);
      });
    });
  }

  // Create a tab button element
  function createTabButton(name, index, containerIndex) {
    const button = document.createElement('button');
    button.className = 'tab-button';
    button.textContent = name;
    button.setAttribute('role', 'tab');
    button.setAttribute('aria-selected', 'false');
    button.setAttribute('aria-controls', `tabpanel-${containerIndex}-${index}`);
    button.setAttribute('id', `tab-${containerIndex}-${index}`);
    button.setAttribute('tabindex', '-1');
    return button;
  }

  // Activate a specific tab
  function activateTab(container, index) {
    const buttons = container.querySelectorAll('.tab-button');
    const panes = container.querySelectorAll('.tab-pane');

    // Deactivate all
    buttons.forEach(btn => {
      btn.classList.remove('active');
      btn.setAttribute('aria-selected', 'false');
      btn.setAttribute('tabindex', '-1');
    });

    panes.forEach(pane => {
      pane.classList.remove('active');
    });

    // Activate selected
    if (buttons[index] && panes[index]) {
      buttons[index].classList.add('active');
      buttons[index].setAttribute('aria-selected', 'true');
      buttons[index].setAttribute('tabindex', '0');
      panes[index].classList.add('active');
    }
  }

  // Handle keyboard navigation
  function handleKeyboard(e, buttons) {
    const currentIndex = Array.from(buttons).findIndex(btn => btn === document.activeElement);
    if (currentIndex === -1) return;

    let newIndex = currentIndex;

    switch (e.key) {
      case 'ArrowLeft':
        newIndex = currentIndex > 0 ? currentIndex - 1 : buttons.length - 1;
        e.preventDefault();
        break;
      case 'ArrowRight':
        newIndex = currentIndex < buttons.length - 1 ? currentIndex + 1 : 0;
        e.preventDefault();
        break;
      case 'Home':
        newIndex = 0;
        e.preventDefault();
        break;
      case 'End':
        newIndex = buttons.length - 1;
        e.preventDefault();
        break;
      default:
        return;
    }

    buttons[newIndex].focus();
    buttons[newIndex].click();
  }

  // Initialize on DOM ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initTabs);
  } else {
    initTabs();
  }

  // Re-initialize if content is dynamically loaded
  if (typeof MutationObserver !== 'undefined') {
    const observer = new MutationObserver((mutations) => {
      let shouldUpdate = false;
      mutations.forEach((mutation) => {
        mutation.addedNodes.forEach((node) => {
          if (node.nodeType === 1 && (node.hasAttribute('data-tabs') || node.querySelector('[data-tabs]'))) {
            shouldUpdate = true;
          }
        });
      });
      if (shouldUpdate) {
        initTabs();
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }
})();
