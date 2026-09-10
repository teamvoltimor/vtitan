"""Accuracy evaluation for compiled models.

``compare_hars.py`` scores quantized HARs inside the suite container;
``float_anchor.py`` scores the float checkpoint on the host to give those
numbers a ceiling. Both share ``metrics.py`` so a difference between them can
only come from the model, never from the scoring.
"""
