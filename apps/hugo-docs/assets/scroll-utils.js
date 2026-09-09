(function() {
  'use strict';

  // Configuration constants
  const BACK_TO_TOP_THRESHOLD = 300;

  // DOM element references
  let backToTopButton = null;

  // Initialize elements
  function initElements() {
    backToTopButton = document.getElementById('back-to-top');
  }

  // Update back-to-top button visibility
  function updateBackToTop() {
    const scrollY = window.scrollY || window.pageYOffset;

    if (backToTopButton) {
      backToTopButton.style.display = scrollY > BACK_TO_TOP_THRESHOLD ? 'flex' : 'none';
    }
  }

  // Scroll to top
  function scrollToTop() {
    window.scrollTo({ top: 0, behavior: 'smooth' });
  }

  // Setup event listeners
  function setupListeners() {
    if (backToTopButton) {
      backToTopButton.addEventListener('click', scrollToTop);

      backToTopButton.addEventListener('keydown', function(event) {
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          scrollToTop();
        }
      });
    }
  }

  // Initialize
  function init() {
    initElements();
    setupListeners();

    window.addEventListener('scroll', updateBackToTop, { passive: true });
    window.addEventListener('resize', updateBackToTop, { passive: true });
    document.addEventListener('DOMContentLoaded', updateBackToTop);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
