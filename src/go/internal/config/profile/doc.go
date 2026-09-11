// Package profile loads per-component hardware profiles (viper-backed),
// mirroring the existing Python hardware-profile-per-component system
// (src/python/shared/src/shared/config/hardware_profile.py and
// platform/robot/src/hardware/settings_base.py): a base TOML file plus zero
// or more named profile overlays, deep-merged in order via
// viper.MergeInConfig, later profiles winning.
package profile
