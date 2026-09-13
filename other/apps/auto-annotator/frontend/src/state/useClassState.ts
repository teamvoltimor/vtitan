import { useCallback, useState } from 'react';
import { type ClassItem, upsertClass } from '../api/client';

/**
 * Owns the annotation class list, per-class colors, and the currently
 * active class. `recordAction` is injected so this slice stays decoupled
 * from the activity-log slice.
 */
export const useClassState = (recordAction: (label: string) => void) => {
  const [activeClass, setActiveClass] = useState<string | null>(null);
  const [classes, setClasses] = useState<string[]>([]);
  const [classColors, setClassColors] = useState<Record<string, string>>({});

  const applyClasses = useCallback((items: ClassItem[]) => {
    setClasses(items.map((c) => c.name));
    setClassColors(Object.fromEntries(items.map((c) => [c.name, c.color])));
  }, []);

  const addClass = useCallback(
    async (name: string, color: string) => {
      const items = await upsertClass(name, color);
      applyClasses(items);
      setActiveClass(name);
      recordAction(`Created class ${name}`);
    },
    [applyClasses, recordAction]
  );

  const updateClassColor = useCallback(
    async (name: string, color: string) => {
      const items = await upsertClass(name, color);
      applyClasses(items);
      recordAction(`Updated color for ${name}`);
    },
    [applyClasses, recordAction]
  );

  return {
    activeClass,
    setActiveClass,
    classes,
    classColors,
    addClass,
    updateClassColor,
    applyClasses,
  };
};
