package hwconfig

import (
	"fmt"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// NATSFaults resolves the NATS fault plan from configRoot's
// nats_faults.toml, overlaid with the named profiles. Set it on every
// nats.Config a process connects with, so each of its subscribers sees the
// same plan.
//
// An empty configRoot is no faults. A load failure or an invalid value is an
// error rather than a fallback, because a run silently made fault-free
// would look better than the one asked for.
func NATSFaults(configRoot string, profiles []string) (nats.Faults, error) {
	if configRoot == "" {
		return nats.Faults{}, nil
	}

	loaded, err := profile.Load[hardware.HardwareNatsFaults](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultNATSFaultsTOMLPath)),
		profiles,
	)
	if err != nil {
		return nats.Faults{}, fmt.Errorf("nats faults: loading nats_faults.toml: %w", err)
	}
	if loaded.Seed < 0 {
		return nats.Faults{}, fmt.Errorf("nats faults: seed %d must not be negative", loaded.Seed)
	}

	faults := nats.Faults{Seed: uint64(loaded.Seed)}
	for _, s := range loaded.Subjects {
		if _, dup := faults.Subjects[s.Subject]; dup {
			return nats.Faults{}, fmt.Errorf("nats faults: subject %s is listed twice", s.Subject)
		}
		if faults.Subjects == nil {
			faults.Subjects = make(map[string]nats.SubjectFaults, len(loaded.Subjects))
		}
		faults.Subjects[s.Subject] = subjectFaults(s)
	}
	// The TOML decoder does not apply the schema's bounds, so check here.
	if err = faults.Validate(); err != nil {
		return nats.Faults{}, fmt.Errorf("nats faults: %w", err)
	}
	return faults, nil
}

func subjectFaults(s hardware.HardwareNatsFaultsSubjectsElem) nats.SubjectFaults {
	burst := s.DropBurst
	if burst == 0 {
		// The schema's default: the decoder leaves an omitted key at zero.
		burst = 1
	}
	return nats.SubjectFaults{
		NoiseFields:    s.NoiseFields,
		Delay:          millis(s.DelayMs),
		Jitter:         millis(s.JitterMs),
		FreezeDuration: millis(s.FreezeMs),
		DropRate:       s.DropRate,
		DropBurst:      burst,
		FreezeRate:     s.FreezeRateHz,
		ReorderRate:    s.ReorderRate,
		NoiseSigma:     s.NoiseSigma,
	}
}
