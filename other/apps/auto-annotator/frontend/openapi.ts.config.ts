import { defineConfig } from 'openapi-typescript';

export default defineConfig({
  input: '../api/openapi.yaml',
  output: './src/api/types.generated.ts',
  prettier: true,
});
