(function() {
  'use strict';

  const THEMES = ['light', 'dark', 'auto'];
  const STORAGE_KEY = 'theme-preference';

  // Debug logging - enabled via window.themeDebug = true or BookDebug param
  function debug() {
    if (window.themeDebug) {
      console.apply(console, ['[Theme]'].concat(Array.from(arguments)));
    }
  }

  // Get system preference
  function getSystemTheme() {
    if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
      return 'dark';
    }
    return 'light';
  }

  // Get current theme preference from localStorage
  function getStoredTheme() {
    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      return THEMES.includes(stored) ? stored : 'auto';
    } catch (e) {
      return 'auto';
    }
  }

  // Store theme preference
  function storeTheme(theme) {
    try {
      localStorage.setItem(STORAGE_KEY, theme);
    } catch (e) {
      debug('Could not store theme preference:', e);
    }
  }

  // Get effective theme (resolves 'auto' to actual theme)
  function getEffectiveTheme(preference) {
    if (preference === 'auto') {
      return getSystemTheme();
    }
    return preference;
  }

  // Apply theme to document - FORCE it
  function applyTheme(preference) {
    const effectiveTheme = getEffectiveTheme(preference);
    const html = document.documentElement;
    const body = document.body;

    debug('Applying theme:', effectiveTheme, 'from preference:', preference);

    // Remove ALL theme classes first
    html.classList.remove('light', 'dark');
    body.classList.remove('light', 'dark');

    // Apply the new theme class
    html.classList.add(effectiveTheme);
    body.classList.add(effectiveTheme);

    // Set data attributes
    html.setAttribute('data-theme', effectiveTheme);
    html.setAttribute('data-theme-preference', preference);

    // Force style recalculation
    void html.offsetWidth;

    // Update button state
    updateButtonState(preference);
    
    debug('Theme applied. HTML classes:', html.className, 'Body classes:', body.className);
  }

  // Update theme toggle button state
  function updateButtonState(preference) {
    const button = document.getElementById('theme-toggle');
    if (!button) return;

    const icons = button.querySelectorAll('.theme-icon');
    icons.forEach(icon => {
      icon.style.display = 'none';
    });

    const activeIcon = button.querySelector(`.theme-icon-${preference}`);
    if (activeIcon) {
      activeIcon.style.display = 'block';
    }

    button.setAttribute('aria-label', `Current theme: ${preference}. Click to change.`);
    button.setAttribute('title', `Theme: ${preference} (click to change)`);
  }

  // Cycle to next theme
  function cycleTheme(event) {
    if (event) {
      event.preventDefault();
    }

    const currentPreference = getStoredTheme();
    const currentIndex = THEMES.indexOf(currentPreference);
    const nextIndex = (currentIndex + 1) % THEMES.length;
    const nextTheme = THEMES[nextIndex];

    debug('Cycling from', currentPreference, 'to', nextTheme);

    storeTheme(nextTheme);
    applyTheme(nextTheme);
  }

  // Listen for system theme changes when in auto mode
  function watchSystemTheme() {
    if (!window.matchMedia) return;

    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');

    const handleChange = () => {
      const preference = getStoredTheme();
      if (preference === 'auto') {
        debug('System theme changed, reapplying auto theme');
        applyTheme('auto');
      }
    };

    if (mediaQuery.addEventListener) {
      mediaQuery.addEventListener('change', handleChange);
    } else if (mediaQuery.addListener) {
      mediaQuery.addListener(handleChange);
    }
  }

  // Setup event listeners
  function setupListeners() {
    const button = document.getElementById('theme-toggle');
    if (button) {
      button.addEventListener('click', cycleTheme);
      debug('Theme toggle button listener attached');
    } else {
      debug('Theme toggle button not found!');
    }
  }

  // Initialize on DOM ready
  function init() {
    const preference = getStoredTheme();
    debug('Initializing theme system with preference:', preference);
    applyTheme(preference);
    setupListeners();
    watchSystemTheme();
  }

  // Run when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
