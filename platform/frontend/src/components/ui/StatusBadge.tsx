/** Online/offline pill badge. */
export function StatusBadge({ online, label }: { online: boolean; label: string }) {
  return (
    <span className={`status-badge ${online ? 'online' : 'offline'}`}>{label}</span>
  )
}
