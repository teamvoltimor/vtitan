(function() {
  'use strict';

  // Configuration constants
  const TAB_CONTAINER_SELECTOR = '[data-tabs]';
  const TAB_BUTTON_CLASS = 'tab-button';
  const TAB_PANE_CLASS = 'tab-pane';
  const DEFAULT_TAB_NAME_PREFIX = 'Tab ';

  // Check if node is a tab container
  function isTabContainer(node) {
    return node.nodeType === 1 && (
      node.hasAttribute('data-tabs') ||
      node.querySelector('[data-tabs]')
    );
  }

  // Check if mutations contain new tab containers
  function hasNewTabContainers(mutations) {
    return mutations.some((mutation) => {
      return Array.from(mutation.addedNodes).some(isTabContainer);
    });
  }

  // Create tab button element
  function createTabButton(name, index, containerIndex) {
    const button = document.createElement('button');
    button.className = TAB_BUTTON_CLASS;
    button.textContent = name;
    button.setAttribute('role', 'tab');
    button.setAttribute('aria-selected', 'false');
    button.setAttribute('aria-controls', `tabpanel-${containerIndex}-${index}`);
    button.setAttribute('id', `tab-${containerIndex}-${index}`);
    button.setAttribute('tabindex', '-1');
    return button;
  }

  // Get tab name from pane
  function getTabName(pane, index) {
    return pane.getAttribute('data-tab-name') || (DEFAULT_TAB_NAME_PREFIX + (index + 1));
  }

  // Find initial tab index from URL hash
  function findInitialTab(tabPanes) {
    const hash = window.location.hash.slice(1);
    if (!hash) return 0;

    const hashIndex = Array.from(tabPanes).findIndex((pane) => pane.id === hash);
    return hashIndex !== -1 ? hashIndex : 0;
  }

  // Activate a specific tab
  function activateTab(container, index) {
    const buttons = container.querySelectorAll('.' + TAB_BUTTON_CLASS);
    const panes = container.querySelectorAll('.' + TAB_PANE_CLASS);

    // Deactivate all
    buttons.forEach((btn) => {
      btn.classList.remove('active');
      btn.setAttribute('aria-selected', 'false');
      btn.setAttribute('tabindex', '-1');
    });

    panes.forEach((pane) => {
      pane.classList.remove('active');
    });

    // Activate selected
    if (!buttons[index] || !panes[index]) return;

    buttons[index].classList.add('active');
    buttons[index].setAttribute('aria-selected', 'true');
    buttons[index].setAttribute('tabindex', '0');
    panes[index].classList.add('active');
  }

  // Handle tab click
  function handleTabClick(container, index) {
    return () => {
      activateTab(container, index);
      const panes = container.querySelectorAll('.' + TAB_PANE_CLASS);
      if (panes[index].id) {
        history.pushState(null, null, '#' + panes[index].id);
      }
    };
  }

  // Handle keyboard navigation
  function handleKeyboard(event, buttons) {
    const currentIndex = Array.from(buttons).findIndex((btn) => btn === document.activeElement);
    if (currentIndex === -1) return;

    const keyActions = {
      ArrowLeft: currentIndex > 0 ? currentIndex - 1 : buttons.length - 1,
      ArrowRight: currentIndex < buttons.length - 1 ? currentIndex + 1 : 0,
      Home: 0,
      End: buttons.length - 1
    };

    const newIndex = keyActions[event.key];
    if (newIndex === undefined) return;

    event.preventDefault();
    buttons[newIndex].focus();
    buttons[newIndex].click();
  }

  // Initialize tabs for a single container
  function initContainer(container, containerIndex) {
    const tabPanes = container.querySelectorAll('.' + TAB_PANE_CLASS);
    const tabsHeader = container.querySelector('.tabs-header');

    if (!tabsHeader || tabPanes.length === 0) return;

    // Create tab buttons
    const buttons = [];
    tabPanes.forEach((pane, index) => {
      const tabName = getTabName(pane, index);
      const button = createTabButton(tabName, index, containerIndex);
      buttons.push(button);
      tabsHeader.appendChild(button);
    });

    const initialTab = findInitialTab(tabPanes);
    activateTab(container, initialTab);

    // Add click handlers
    buttons.forEach((button, index) => {
      button.addEventListener('click', handleTabClick(container, index));
    });

    // Keyboard navigation
    tabsHeader.addEventListener('keydown', (event) => handleKeyboard(event, buttons));
  }

  // Initialize all tabs on the page
  function initTabs() {
    const tabContainers = document.querySelectorAll(TAB_CONTAINER_SELECTOR);
    tabContainers.forEach(initContainer);
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
      if (hasNewTabContainers(mutations)) {
        initTabs();
      }
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }
})();
