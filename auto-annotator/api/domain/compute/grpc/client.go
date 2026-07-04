// Package grpc is the gRPC adapter for the compute domain.
package grpc

import (
	"context"
	"errors"
	"fmt"
	"io"

	xgrpc "google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute"
	computev1 "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute/grpc/pb/autoannotator/v1"
)

type grpcClients struct {
	seg   computev1.SegmentationServiceClient
	aug   computev1.AugmentationServiceClient
	train computev1.TrainingServiceClient
	conns []*xgrpc.ClientConn
}

// NewGRPC dials the configured worker addresses. Empty addresses leave that worker disabled.
func NewGRPC(segAddr, augAddr, trainAddr string) (compute.Clients, error) {
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

func dial(addr string) (*xgrpc.ClientConn, error) {
	return xgrpc.NewClient(addr, xgrpc.WithTransportCredentials(insecure.NewCredentials()))
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

func (g *grpcClients) Segment(ctx context.Context, in compute.SegmentInput) (compute.SegmentResult, error) {
	if g.seg == nil {
		return compute.SegmentResult{}, errors.New("segmentation worker not configured")
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
		return compute.SegmentResult{}, err
	}
	return compute.SegmentResult{
		State:   resp.GetState(),
		Message: resp.GetMessage(),
		Shapes:  fromPbShapes(resp.GetShapes()),
	}, nil
}

func (g *grpcClients) RunAugmentation(ctx context.Context, in compute.AugmentInput, onProgress func(compute.Progress)) error {
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

func (g *grpcClients) RunTraining(ctx context.Context, in compute.TrainInput, onProgress func(compute.Progress)) error {
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

func relayProgress(stream xgrpc.ServerStreamingClient[computev1.JobProgress], onProgress func(compute.Progress)) error {
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

func fromPbProgress(p *computev1.JobProgress) compute.Progress {
	out := compute.Progress{
		Status:   p.GetStatus(),
		Message:  p.GetMessage(),
		Stage:    p.GetStage(),
		Progress: p.GetProgress(),
		Details:  p.GetDetails(),
		Finished: p.GetFinished(),
		Error:    p.GetError(),
	}
	if a := p.GetAugmented(); a != nil {
		out.Augmented = &compute.AugmentedImage{Path: a.GetPath(), FormatUsed: a.GetFormatUsed(), ParentID: a.GetParentId()}
	}
	return out
}

func fromPbShapes(in []*computev1.Shape) []compute.Shape {
	shapes := make([]compute.Shape, 0, len(in))
	for _, s := range in {
		pts := make([]compute.Point, 0, len(s.GetPoints()))
		for _, p := range s.GetPoints() {
			pts = append(pts, compute.Point{X: p.GetX(), Y: p.GetY()})
		}
		shapes = append(shapes, compute.Shape{ID: s.GetId(), ClassName: s.GetClassName(), Points: pts})
	}
	return shapes
}
