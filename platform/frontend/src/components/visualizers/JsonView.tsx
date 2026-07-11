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
    if (data.length > JSON_VIEW_CONFIG.ARRAY_PREVIEW_LIMIT) {
      return (
        <span className="json-array">
          [Array({data.length})] {data.slice(0, 3).map(String).join(', ')}...
        </span>
      );
    }
    return (
      <span className="json-array">
        [
        {data.map((item, i) => (
          // biome-ignore lint/suspicious/noArrayIndexKey: arbitrary JSON array of primitives/objects with no stable id; position is the only available identity
          <span key={i}>
            <JsonView data={item} level={level + 1} />
            {i < data.length - 1 && ', '}
          </span>
        ))}
        ]
      </span>
    );
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
