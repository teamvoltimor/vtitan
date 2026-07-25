import type { ReactNode } from 'react';
import { Card, Stack, Typography } from '@mui/material';

export function SectionCard({
  title,
  children,
  action,
}: {
  title?: string;
  children: ReactNode;
  action?: ReactNode;
}) {
  return (
    <Card variant="outlined" sx={{ p: 2 }}>
      <Stack spacing={2}>
        {title && (
          <Stack direction="row" justifyContent="space-between" alignItems="center">
            <Typography variant="subtitle2" sx={{ fontWeight: 600 }}>
              {title}
            </Typography>
            {action}
          </Stack>
        )}
        {children}
      </Stack>
    </Card>
  );
}
