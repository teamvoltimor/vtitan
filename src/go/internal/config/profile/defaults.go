package profile

import (
	"fmt"
	"reflect"
	"strconv"
	"strings"
)

// tagDefaults reflect-walks T and builds the viper.SetDefault map from each
// field's `default:"..."` struct tag, keyed by its `mapstructure` name and
// dotted for nested structs.
//
// This is what lets a TOML key some source files never set fall back to the
// shipped value without a parallel map[string]any: the default sits on the
// field it belongs to, beside the same `mapstructure` tag mapstructure decodes
// by, so the two cannot drift. A field with no `default` tag contributes
// nothing and an omitted key reads as the Go zero value, exactly as before.
func tagDefaults[T any]() (map[string]any, error) {
	defaults := map[string]any{}
	if err := collectDefaults(reflect.TypeFor[T](), "", defaults); err != nil {
		return nil, err
	}
	return defaults, nil
}

// collectDefaults walks value type t under the dotted key prefix, recording
// each tagged scalar in out. Unexported fields and `mapstructure:"-"` are
// skipped, matching how viper/mapstructure address the decoded struct; a
// pointer's pointee is what carries the kind to parse.
func collectDefaults(t reflect.Type, prefix string, out map[string]any) error {
	for field := range t.Fields() {
		if !field.IsExported() {
			continue
		}
		name := mapstructureName(field)
		if name == "" {
			continue
		}
		key := name
		if prefix != "" {
			key = prefix + "." + name
		}
		if err := collectField(field, key, out); err != nil {
			return err
		}
	}
	return nil
}

// collectField records the `default` tag of one field under key, or recurses
// into it when it is an untagged struct (a nested TOML table).
func collectField(field reflect.StructField, key string, out map[string]any) error {
	valueType := field.Type
	for valueType.Kind() == reflect.Pointer {
		valueType = valueType.Elem()
	}

	raw := field.Tag.Get("default")
	if raw == "" {
		if valueType.Kind() == reflect.Struct {
			return collectDefaults(valueType, key, out)
		}
		return nil
	}

	value, err := parseDefault(valueType, raw)
	if err != nil {
		return fmt.Errorf("profile: %s: %w", key, err)
	}
	out[key] = value
	return nil
}

// mapstructureName returns the TOML key a field decodes from: the name half of
// its `mapstructure` tag, or the lowercased field name (viper's own
// case-insensitive fallback) when that tag carries no name. An explicit "-"
// means the field is not addressable and yields "".
func mapstructureName(field reflect.StructField) string {
	tag := field.Tag.Get("mapstructure")
	if tag == "-" {
		return ""
	}
	if name, _, _ := strings.Cut(tag, ","); name != "" {
		return name
	}
	return strings.ToLower(field.Name)
}

// parseDefault parses a `default` tag into the field's own type, so the value
// viper stores has the exact kind mapstructure will assign rather than a
// generic int64/float64 viper would have to coerce back.
func parseDefault(valueType reflect.Type, raw string) (any, error) {
	value := reflect.New(valueType).Elem()
	switch valueType.Kind() {
	case reflect.Bool:
		parsed, err := strconv.ParseBool(raw)
		if err != nil {
			return nil, fmt.Errorf("parsing bool default %q: %w", raw, err)
		}
		value.SetBool(parsed)
	case reflect.String:
		value.SetString(raw)
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		parsed, err := strconv.ParseInt(raw, 10, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing int default %q: %w", raw, err)
		}
		value.SetInt(parsed)
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		parsed, err := strconv.ParseUint(raw, 10, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing uint default %q: %w", raw, err)
		}
		value.SetUint(parsed)
	case reflect.Float32, reflect.Float64:
		parsed, err := strconv.ParseFloat(raw, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing float default %q: %w", raw, err)
		}
		value.SetFloat(parsed)
	default:
		return nil, fmt.Errorf("unsupported default type %s", valueType)
	}
	return value.Interface(), nil
}
