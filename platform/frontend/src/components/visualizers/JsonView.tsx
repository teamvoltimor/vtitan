import { useState } from 'react';
import { formatNumber } from '../../utils/formatting';
import { JSON_VIEW_CONFIG } from '../../config';

/** Recursive, syntax-highlighted view of arbitrary topic data. */
export function JsonView({ data, level = 0 }: { data: unknown; level?: number }) {
  if (data === null || data === undefined) {
    return <span className="json-null">null</span>;
  }

  if (typeof data === 'number') {
    return (
      <span className="json-number">
        {formatNumber(data, { decimals: JSON_VIEW_CONFIG.DECIMAL_PRECISION })}
      </span>
    );
  }

  if (typeof data === 'string') {
    return <span className="json-string">"{data}"</span>;
  }

  if (typeof data === 'boolean') {
    return <span className="json-boolean">{data.toString()}</span>;
  }

  if (Array.isArray(data)) {
    return <JsonArrayView data={data} level={level} />;
  }

  if (typeof data === 'object') {
    const entries = Object.entries(data as Record<string, unknown>);
    return (
      <div
        className="json-object"
        style={{ marginLeft: `${level * JSON_VIEW_CONFIG.INDENT_PER_LEVEL}px` }}
      >
        {entries.map(([key, value]) => (
          <div key={key} className="json-field">
            <span className="json-key">{key}:</span> <JsonView data={value} level={level + 1} />
          </div>
        ))}
      </div>
    );
  }

  return <span>{String(data)}</span>;
}

/**
 * Array rendering with a "show N more" control instead of a dead-end
 * stringified preview — long arrays of objects previously collapsed to
 * unreadable `[object Object]` text with no way to expand.
 */
function JsonArrayView({ data, level }: { data: unknown[]; level: number }) {
  const [visibleCount, setVisibleCount] = useState<number>(JSON_VIEW_CONFIG.ARRAY_PREVIEW_LIMIT);
  const visible = data.slice(0, visibleCount);
  const remaining = data.length - visible.length;

  return (
    <span className="json-array">
      [
      {visible.map((item, i) => (
        // biome-ignore lint/suspicious/noArrayIndexKey: arbitrary JSON array of primitives/objects with no stable id; position is the only available identity
        <span key={i}>
          <JsonView data={item} level={level + 1} />
          {(i < visible.length - 1 || remaining > 0) && ', '}
        </span>
      ))}
      {remaining > 0 && (
        <button
          type="button"
          className="json-array-more"
          onClick={() => setVisibleCount((count) => count + JSON_VIEW_CONFIG.ARRAY_EXPAND_STEP)}
        >
          show {Math.min(remaining, JSON_VIEW_CONFIG.ARRAY_EXPAND_STEP)} more ({remaining} left)
        </button>
      )}
      ]
    </span>
  );
}
