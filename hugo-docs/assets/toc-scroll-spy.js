(function() {
  'use strict';

  const TOC_SELECTOR = '.book-toc nav a';
  const HEADING_SELECTOR = 'h1[id], h2[id], h3[id], h4[id], h5[id], h6[id]';
  const ACTIVE_CLASS = 'active';
  const OFFSET = 100; // Offset from top to trigger active state

  let tocLinks = [];
  let headings = [];
  let currentActive = null;

  // Initialize scroll spy
  function init() {
    tocLinks = Array.from(document.querySelectorAll(TOC_SELECTOR));
    if (tocLinks.length === 0) return;

    // Get all headings with IDs
    headings = Array.from(document.querySelectorAll(HEADING_SELECTOR))
      .map(heading => ({
        id: heading.id,
        element: heading,
        top: 0
      }));

    if (headings.length === 0) return;

    // Update heading positions
    updateHeadingPositions();

    // Set up scroll listener
    window.addEventListener('scroll', onScroll, { passive: true });
    window.addEventListener('resize', updateHeadingPositions, { passive: true });

    // Initial check
    onScroll();
  }

  // Update heading positions (on load and resize)
  function updateHeadingPositions() {
    headings.forEach(heading => {
      heading.top = heading.element.getBoundingClientRect().top + window.pageYOffset;
    });
  }

  // Handle scroll event
  function onScroll() {
    const scrollPos = window.pageYOffset + OFFSET;

    // Find the current heading
    let currentHeading = null;
    for (let i = headings.length - 1; i >= 0; i--) {
      if (scrollPos >= headings[i].top) {
        currentHeading = headings[i];
        break;
      }
    }

    // If we're at the very top, deactivate all
    if (window.pageYOffset < 50) {
      currentHeading = null;
    }

    // Update active link
    updateActiveLink(currentHeading ? currentHeading.id : null);
  }

  // Update the active TOC link
  function updateActiveLink(targetId) {
    // Skip if already active
    if (currentActive === targetId) return;

    // Remove previous active class
    tocLinks.forEach(link => link.classList.remove(ACTIVE_CLASS));

    // Add active class to current link
    if (targetId) {
      const activeLink = tocLinks.find(link => {
        const href = link.getAttribute('href');
        return href === `#${targetId}`;
      });

      if (activeLink) {
        activeLink.classList.add(ACTIVE_CLASS);
        scrollTocToView(activeLink);
      }
    }

    currentActive = targetId;
  }

  // Scroll TOC to keep active item visible
  function scrollTocToView(link) {
    const tocContainer = link.closest('.book-toc-content');
    if (!tocContainer) return;

    const linkRect = link.getBoundingClientRect();
    const containerRect = tocContainer.getBoundingClientRect();

    // Check if link is out of view
    const isAbove = linkRect.top < containerRect.top;
    const isBelow = linkRect.bottom > containerRect.bottom;

    if (isAbove || isBelow) {
      const scrollTop = link.offsetTop - tocContainer.offsetTop - (containerRect.height / 2) + (linkRect.height / 2);
      tocContainer.scrollTo({
        top: scrollTop,
        behavior: 'smooth'
      });
    }
  }

  // Smooth scroll to heading when TOC link is clicked
  function addSmoothScroll() {
    tocLinks.forEach(link => {
      link.addEventListener('click', (e) => {
        const href = link.getAttribute('href');
        if (!href || !href.startsWith('#')) return;

        const targetId = href.slice(1);
        const target = document.getElementById(targetId);
        if (!target) return;

        e.preventDefault();

        // Scroll to target
        const targetTop = target.getBoundingClientRect().top + window.pageYOffset - 80;
        window.scrollTo({
          top: targetTop,
          behavior: 'smooth'
        });

        // Update URL hash without jumping
        if (history.pushState) {
          history.pushState(null, null, href);
        } else {
          location.hash = href;
        }
      });
    });
  }

  // Start when DOM is ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      init();
      addSmoothScroll();
    });
  } else {
    init();
    addSmoothScroll();
  }
})();
