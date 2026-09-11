import { Chip, Stack, Typography } from '@mui/material';

export function ChipGroup<T extends string>({
  options,
  selected,
  onChange,
  label,
}: {
  options: readonly T[] | T[];
  selected: T;
  onChange: (value: T) => void;
  label?: string;
}) {
  return (
    <Stack spacing={0.5}>
      {label && (
        <Typography variant="caption" color="text.disabled">
          {label}
        </Typography>
      )}
      <Stack direction="row" spacing={0.5} flexWrap="wrap">
        {options.map((option) => (
          <Chip
            key={option}
            label={option}
            size="small"
            variant={selected === option ? 'filled' : 'outlined'}
            color={selected === option ? 'primary' : 'default'}
            onClick={() => onChange(option)}
            sx={{ cursor: 'pointer', fontSize: '0.65rem', mb: 0.5 }}
          />
        ))}
      </Stack>
    </Stack>
  );
}
