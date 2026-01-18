(function() {
  'use strict';

  const button = document.getElementById('back-to-top');
  const progressBar = document.getElementById('back-to-top-progress-bar');

  if (!button || !progressBar) return;

  const SHOW_THRESHOLD = 300; // Show button after scrolling 300px
  const CIRCLE_LENGTH = 62.83; // 2 * PI * radius (radius = 10)

  // Show/hide button and update progress
  function updateButton() {
    const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
    const docHeight = document.documentElement.scrollHeight - document.documentElement.clientHeight;
    const scrollPercent = scrollTop / docHeight;

    // Show/hide button
    if (scrollTop > SHOW_THRESHOLD) {
      button.style.display = 'flex';
      // Fade in animation
      requestAnimationFrame(() => {
        button.style.opacity = '1';
      });
    } else {
      button.style.opacity = '0';
      setTimeout(() => {
        if (button.style.opacity === '0') {
          button.style.display = 'none';
        }
      }, 200);
    }

    // Update progress circle
    const offset = CIRCLE_LENGTH - (scrollPercent * CIRCLE_LENGTH);
    progressBar.style.strokeDashoffset = offset;
  }

  // Scroll to top smoothly
  function scrollToTop(e) {
    e.preventDefault();
    window.scrollTo({
      top: 0,
      behavior: 'smooth'
    });
  }

  // Event listeners
  window.addEventListener('scroll', updateButton, { passive: true });
  button.addEventListener('click', scrollToTop);

  // Initialize
  button.style.opacity = '0';
  button.style.transition = 'opacity 0.2s';
  updateButton();
})();
