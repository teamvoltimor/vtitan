(function() {
  'use strict';

  // Configuration constants
  const THEMES = ['light', 'dark', 'auto'];
  const STORAGE_KEY = 'theme-preference';
  const DARK_MODE_MEDIA_QUERY = '(prefers-color-scheme: dark)';
  const THEME_TOGGLE_ID = 'theme-toggle';
  const THEME_ICON_CLASS = 'theme-icon';

  // Debug logging - enabled via window.themeDebug = true or BookDebug param
  function debug() {
    if (window.themeDebug) {
      console.apply(console, ['[Theme]'].concat(Array.from(arguments)));
    }
  }

  // Check if dark mode is preferred by system
  function isSystemDarkMode() {
    return window.matchMedia && window.matchMedia(DARK_MODE_MEDIA_QUERY).matches;
  }

  // Get system preference
  function getSystemTheme() {
    return isSystemDarkMode() ? 'dark' : 'light';
  }

  // Validate theme against allowed themes
  function isValidTheme(theme) {
    return THEMES.includes(theme);
  }

  // Get current theme preference from localStorage
  function getStoredTheme() {
    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      return isValidTheme(stored) ? stored : 'auto';
    } catch {
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
    return preference === 'auto' ? getSystemTheme() : preference;
  }

  // Apply theme to document
  function applyTheme(preference) {
    const effectiveTheme = getEffectiveTheme(preference);
    const html = document.documentElement;
    const body = document.body;

    debug('Applying theme:', effectiveTheme, 'from preference:', preference);

    // Remove all theme classes
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

    updateButtonState(preference);

    debug('Theme applied. HTML classes:', html.className, 'Body classes:', body.className);
  }

  // Hide all theme icons
  function hideAllIcons(button) {
    const icons = button.querySelectorAll('.' + THEME_ICON_CLASS);
    icons.forEach((icon) => {
      icon.style.display = 'none';
    });
  }

  // Show active theme icon
  function showActiveIcon(button, preference) {
    const activeIcon = button.querySelector('.theme-icon-' + preference);
    if (activeIcon) {
      activeIcon.style.display = 'block';
    }
  }

  // Update theme toggle button state
  function updateButtonState(preference) {
    const button = document.getElementById(THEME_TOGGLE_ID);
    if (!button) return;

    hideAllIcons(button);
    showActiveIcon(button, preference);

    button.setAttribute('aria-label', 'Current theme: ' + preference + '. Click to change.');
    button.setAttribute('title', 'Theme: ' + preference + ' (click to change)');
  }

  // Get next theme in cycle
  function getNextTheme(currentTheme) {
    const currentIndex = THEMES.indexOf(currentTheme);
    const nextIndex = (currentIndex + 1) % THEMES.length;
    return THEMES[nextIndex];
  }

  // Cycle to next theme
  function cycleTheme(event) {
    if (event) {
      event.preventDefault();
    }

    const currentPreference = getStoredTheme();
    const nextTheme = getNextTheme(currentPreference);

    debug('Cycling from', currentPreference, 'to', nextTheme);

    storeTheme(nextTheme);
    applyTheme(nextTheme);
  }

  // Handle system theme change
  function handleSystemThemeChange() {
    const preference = getStoredTheme();
    if (preference === 'auto') {
      debug('System theme changed, reapplying auto theme');
      applyTheme('auto');
    }
  }

  // Listen for system theme changes when in auto mode
  function watchSystemTheme() {
    if (!window.matchMedia) return;

    const mediaQuery = window.matchMedia(DARK_MODE_MEDIA_QUERY);

    if (mediaQuery.addEventListener) {
      mediaQuery.addEventListener('change', handleSystemThemeChange);
    } else if (mediaQuery.addListener) {
      mediaQuery.addListener(handleSystemThemeChange);
    }
  }

  // Setup event listeners
  function setupListeners() {
    const button = document.getElementById(THEME_TOGGLE_ID);
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
