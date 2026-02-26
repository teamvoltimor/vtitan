(function() {
  'use strict';

  // Configuration constants
  const BACK_TO_TOP_THRESHOLD = 300;
  const CIRCLE_RADIUS = 10;
  const CIRCLE_CIRCUMFERENCE = 2 * Math.PI * CIRCLE_RADIUS;

  // DOM element references
  let backToTopButton = null;
  let progressBarCircle = null;
  let progressBarLine = null;

  // Initialize elements
  function initElements() {
    backToTopButton = document.getElementById('back-to-top');
    progressBarCircle = document.getElementById('back-to-top-progress-bar');
    progressBarLine = document.getElementById('reading-progress-bar');
  }

  // Update back-to-top button visibility and progress
  function updateBackToTop() {
    const scrollY = window.scrollY || window.pageYOffset;
    const docHeight = document.documentElement.scrollHeight - window.innerHeight;
    const percent = docHeight > 0 ? Math.min(scrollY / docHeight, 1) : 0;

    if (backToTopButton) {
      backToTopButton.style.display = scrollY > BACK_TO_TOP_THRESHOLD ? 'flex' : 'none';
    }

    if (progressBarCircle) {
      const offset = CIRCLE_CIRCUMFERENCE * (1 - percent);
      progressBarCircle.setAttribute('stroke-dashoffset', offset);
    }
  }

  // Scroll to top
  function scrollToTop() {
    window.scrollTo({ top: 0, behavior: 'smooth' });
  }

  // Update reading progress bar (line style)
  function updateProgressLine() {
    const scrollY = window.scrollY || window.pageYOffset;
    const docHeight = document.documentElement.scrollHeight - window.innerHeight;
    const percent = docHeight > 0 ? Math.min(scrollY / docHeight, 1) : 0;

    if (progressBarLine) {
      progressBarLine.style.backgroundSize = (percent * 100) + '% 100%';
      progressBarLine.hidden = docHeight < 100;
    }
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

    if (progressBarLine) {
      window.addEventListener('scroll', updateProgressLine, { passive: true });
      window.addEventListener('resize', updateProgressLine, { passive: true });
      document.addEventListener('DOMContentLoaded', updateProgressLine);
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
