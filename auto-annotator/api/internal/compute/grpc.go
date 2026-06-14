package compute

import (
	"context"
	"errors"
	"fmt"
	"io"

	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"

	computev1 "github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/compute/pb/autoannotator/v1"
)

// grpcClients is the gRPC-backed Clients implementation. A worker whose address
// is unconfigured yields a clear "not configured" error rather than a nil panic,
// so the API degrades gracefully until the Python workers are wired up.
type grpcClients struct {
	seg   computev1.SegmentationServiceClient
	aug   computev1.AugmentationServiceClient
	train computev1.TrainingServiceClient
	conns []*grpc.ClientConn
}

// NewGRPC dials the configured worker addresses (lazily — no connection is
// established until the first RPC). Empty addresses leave that worker disabled.
func NewGRPC(segAddr, augAddr, trainAddr string) (Clients, error) {
	g := &grpcClients{}
	if segAddr != "" {
		conn, err := dial(segAddr)
		if err != nil {
			return nil, fmt.Errorf("dial segmentation worker: %w", err)
		}
		g.conns = append(g.conns, conn)
		g.seg = computev1.NewSegmentationServiceClient(conn)
	}
	if augAddr != "" {
		conn, err := dial(augAddr)
		if err != nil {
			return nil, fmt.Errorf("dial augmentation worker: %w", err)
		}
		g.conns = append(g.conns, conn)
		g.aug = computev1.NewAugmentationServiceClient(conn)
	}
	if trainAddr != "" {
		conn, err := dial(trainAddr)
		if err != nil {
			return nil, fmt.Errorf("dial training worker: %w", err)
		}
		g.conns = append(g.conns, conn)
		g.train = computev1.NewTrainingServiceClient(conn)
	}
	return g, nil
}

func dial(addr string) (*grpc.ClientConn, error) {
	return grpc.NewClient(addr, grpc.WithTransportCredentials(insecure.NewCredentials()))
}

func (g *grpcClients) Close() error {
	var err error
	for _, c := range g.conns {
		if cerr := c.Close(); cerr != nil {
			err = cerr
		}
	}
	return err
}

func (g *grpcClients) Segment(ctx context.Context, in SegmentInput) (SegmentResult, error) {
	if g.seg == nil {
		return SegmentResult{}, errors.New("segmentation worker not configured")
	}
	points := make([]*computev1.ClickPoint, len(in.Points))
	for i, p := range in.Points {
		points[i] = &computev1.ClickPoint{X: p.X, Y: p.Y, PointType: p.PointType, ClassName: p.ClassName}
	}
	resp, err := g.seg.Segment(ctx, &computev1.SegmentRequest{
		ImageId:    in.ImageID,
		ImagePath:  in.ImagePath,
		Points:     points,
		ClassNames: in.ClassNames,
	})
	if err != nil {
		return SegmentResult{}, err
	}
	return SegmentResult{
		State:   resp.GetState(),
		Message: resp.GetMessage(),
		Shapes:  fromPbShapes(resp.GetShapes()),
	}, nil
}

func (g *grpcClients) RunAugmentation(ctx context.Context, in AugmentInput, onProgress func(Progress)) error {
	if g.aug == nil {
		return errors.New("augmentation worker not configured")
	}
	sources := make([]*computev1.AugmentSource, len(in.Sources))
	for i, s := range in.Sources {
		sources[i] = &computev1.AugmentSource{ImageId: s.ImageID, Path: s.Path, FormatUsed: s.FormatUsed}
	}
	stream, err := g.aug.RunAugmentation(ctx, &computev1.AugmentRequest{
		Sources:          sources,
		NumAugmentations: in.NumAugmentations,
	})
	if err != nil {
		return err
	}
	return relayProgress(stream, onProgress)
}

func (g *grpcClients) RunTraining(ctx context.Context, in TrainInput, onProgress func(Progress)) error {
	if g.train == nil {
		return errors.New("training worker not configured")
	}
	stream, err := g.train.RunTraining(ctx, &computev1.TrainRequest{
		ModelName:    in.ModelName,
		Epochs:       in.Epochs,
		Batch:        in.Batch,
		Imgsz:        in.Imgsz,
		DataYamlPath: in.DataYamlPath,
	})
	if err != nil {
		return err
	}
	return relayProgress(stream, onProgress)
}

func relayProgress(stream grpc.ServerStreamingClient[computev1.JobProgress], onProgress func(Progress)) error {
	for {
		msg, err := stream.Recv()
		if errors.Is(err, io.EOF) {
			return nil
		}
		if err != nil {
			return err
		}
		onProgress(fromPbProgress(msg))
	}
}

func fromPbProgress(p *computev1.JobProgress) Progress {
	out := Progress{
		Status:   p.GetStatus(),
		Message:  p.GetMessage(),
		Stage:    p.GetStage(),
		Progress: p.GetProgress(),
		Details:  p.GetDetails(),
		Finished: p.GetFinished(),
		Error:    p.GetError(),
	}
	if a := p.GetAugmented(); a != nil {
		out.Augmented = &AugmentedImage{Path: a.GetPath(), FormatUsed: a.GetFormatUsed(), ParentID: a.GetParentId()}
	}
	return out
}

func fromPbShapes(in []*computev1.Shape) []Shape {
	shapes := make([]Shape, 0, len(in))
	for _, s := range in {
		pts := make([]Point, 0, len(s.GetPoints()))
		for _, p := range s.GetPoints() {
			pts = append(pts, Point{X: p.GetX(), Y: p.GetY()})
		}
		shapes = append(shapes, Shape{ID: s.GetId(), ClassName: s.GetClassName(), Points: pts})
	}
	return shapes
}
