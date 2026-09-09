(function() {
  'use strict';

  let lightbox = null;
  let lightboxImg = null;
  let closeButton = null;
  let lastActiveImg = null;

  // Create lightbox DOM elements
  function createLightbox() {
    lightbox = document.createElement('div');
    lightbox.id = 'image-lightbox';
    lightbox.className = 'image-lightbox';
    lightbox.setAttribute('tabindex', '-1');
    lightbox.setAttribute('aria-modal', 'true');
    lightbox.setAttribute('aria-hidden', 'true');
    lightbox.style.display = 'none';

    const backdrop = document.createElement('div');
    backdrop.className = 'image-lightbox-backdrop';

    lightboxImg = document.createElement('img');
    lightboxImg.className = 'image-lightbox-img';
    lightboxImg.setAttribute('alt', 'Zoomed image');

    closeButton = document.createElement('button');
    closeButton.className = 'image-lightbox-close';
    closeButton.setAttribute('aria-label', 'Close');
    closeButton.innerHTML = '&times;';

    lightbox.appendChild(backdrop);
    lightbox.appendChild(lightboxImg);
    lightbox.appendChild(closeButton);
    document.body.appendChild(lightbox);

    return lightbox;
  }

  // Ensure lightbox exists
  function ensureLightbox() {
    if (lightbox) return;
    createLightbox();
  }

  // Open lightbox
  function openLightbox(src, alt, imgElement) {
    ensureLightbox();

    lightboxImg.src = src;
    lightboxImg.alt = alt || '';
    lightbox.style.display = 'flex';
    lightbox.setAttribute('aria-hidden', 'false');
    lightbox.focus();
    document.body.style.overflow = 'hidden';
    lastActiveImg = imgElement;
  }

  // Close lightbox
  function closeLightbox() {
    if (!lightbox) return;

    lightbox.style.display = 'none';
    lightbox.setAttribute('aria-hidden', 'true');
    lightboxImg.src = '';
    document.body.style.overflow = '';

    if (lastActiveImg) {
      lastActiveImg.focus();
      lastActiveImg = null;
    }
  }

  // Handle image click
  function handleImageClick(event) {
    const img = event.target;
    if (img.tagName === 'IMG' && !img.closest('.image-lightbox')) {
      openLightbox(img.src, img.alt, img);
    }
  }

  // Handle keydown
  function handleKeyDown(event) {
    if (event.key === 'Escape') {
      closeLightbox();
    }
  }

  // Handle lightbox click
  function handleLightboxClick(event) {
    if (event.target === lightbox || event.target.classList.contains('image-lightbox-backdrop')) {
      closeLightbox();
    }
  }

  // Initialize event listeners
  function initListeners() {
    document.body.addEventListener('click', handleImageClick);

    if (closeButton) {
      closeButton.addEventListener('click', closeLightbox);
    }

    if (lightbox) {
      lightbox.addEventListener('click', handleLightboxClick);
      lightbox.addEventListener('keydown', handleKeyDown);
    }
  }

  // Auto-initialize when DOM is ready
  function init() {
    initListeners();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
