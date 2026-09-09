import { Stack } from '@mui/material';
import { useAppState } from '../state/appState';
import { ClassSettings } from './ClassSettings';
import { DisplaySettings } from './DisplaySettings';
import { ModelSettings } from './ModelSettings';

const SettingsTab = () => {
  const {
    models,
    selectedModel,
    modelStatus,
    loadModel,
    classes,
    classColors,
    addClass,
    updateClassColor,
    outlineMode,
    setOutlineMode,
  } = useAppState();

  return (
    <Stack spacing={2.5} sx={{ height: '100%' }}>
      <ModelSettings
        models={models}
        selectedModel={selectedModel}
        modelStatus={modelStatus}
        onModelChange={(id) => void loadModel(id)}
      />

      <ClassSettings
        classes={classes}
        classColors={classColors}
        onUpdateColor={(cls, color) => void updateClassColor(cls, color)}
        onAddClass={(name, color) => void addClass(name, color)}
      />

      <DisplaySettings outlineMode={outlineMode} onOutlineModeChange={setOutlineMode} />
    </Stack>
  );
};

export default SettingsTab;
