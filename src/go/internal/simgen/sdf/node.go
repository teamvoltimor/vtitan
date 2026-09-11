// Package sdf builds and parses Gazebo SDF XML documents.
//
// The central type is Node, a mutable XML element tree that mirrors Python's
// xml.etree.ElementTree.Element API so builder code maps almost 1:1 to the
// original Python builders.
package sdf

import (
	"encoding/xml"
	"fmt"
	"io"
	"strings"
)

// Node is a mutable XML element.
type Node struct {
	Tag      string
	Attrs    [][2]string // ordered [name, value] pairs
	Text     string
	Children []*Node
}

// New creates a new element. attrPairs must be an even-length list of name, value strings.
func New(tag string, attrPairs ...string) *Node {
	n := &Node{Tag: tag}
	for i := 0; i+1 < len(attrPairs); i += 2 {
		n.Attrs = append(n.Attrs, [2]string{attrPairs[i], attrPairs[i+1]})
	}
	return n
}

// T sets the text content and returns the receiver for chaining.
func (n *Node) T(text string) *Node {
	n.Text = text
	return n
}

// Sub creates a child element, appends it, and returns the child.
func (n *Node) Sub(tag string, attrPairs ...string) *Node {
	child := New(tag, attrPairs...)
	n.Children = append(n.Children, child)
	return child
}

// SubT creates a child with text content, appends it, and returns the child.
func (n *Node) SubT(tag, text string) *Node {
	return n.Sub(tag).T(text)
}

// Add appends an existing node as a child and returns the child.
func (n *Node) Add(child *Node) *Node {
	n.Children = append(n.Children, child)
	return child
}

// Insert inserts a child at position idx.
func (n *Node) Insert(idx int, child *Node) {
	n.Children = append(n.Children, nil)
	copy(n.Children[idx+1:], n.Children[idx:])
	n.Children[idx] = child
}

// Find returns the first descendant with the given tag (depth-first), or nil.
func (n *Node) Find(tag string) *Node {
	for _, c := range n.Children {
		if c.Tag == tag {
			return c
		}
		if found := c.Find(tag); found != nil {
			return found
		}
	}
	return nil
}

// FindAttr returns the first descendant matching tag and one attribute name+value, or nil.
func (n *Node) FindAttr(tag, attrName, attrVal string) *Node {
	for _, c := range n.Children {
		if c.Tag == tag {
			for _, a := range c.Attrs {
				if a[0] == attrName && a[1] == attrVal {
					return c
				}
			}
		}
		if found := c.FindAttr(tag, attrName, attrVal); found != nil {
			return found
		}
	}
	return nil
}

// RemoveChild removes the first direct child pointer-equal to child.
func (n *Node) RemoveChild(child *Node) {
	for i, c := range n.Children {
		if c == child {
			n.Children = append(n.Children[:i], n.Children[i+1:]...)
			return
		}
	}
}

// WriteTo serializes the node tree as indented XML with a UTF-8 declaration.
// It implements io.WriterTo.
func (n *Node) WriteTo(w io.Writer) (int64, error) {
	nWritten, err := fmt.Fprint(w, "<?xml version='1.0' encoding='utf-8'?>\n")
	if err != nil {
		return int64(nWritten), err
	}
	enc := xml.NewEncoder(w)
	enc.Indent("", "  ")
	if err = encodeNode(enc, n); err != nil {
		return int64(nWritten), err
	}
	if err = enc.Flush(); err != nil {
		return int64(nWritten), err
	}
	return int64(nWritten), nil
}

func encodeNode(enc *xml.Encoder, n *Node) error {
	start := xml.StartElement{Name: xml.Name{Local: n.Tag}}
	for _, a := range n.Attrs {
		start.Attr = append(start.Attr, xml.Attr{
			Name:  xml.Name{Local: a[0]},
			Value: a[1],
		})
	}
	if err := enc.EncodeToken(start); err != nil {
		return fmt.Errorf("encode start element %q: %w", n.Tag, err)
	}
	if n.Text != "" {
		if err := enc.EncodeToken(xml.CharData(n.Text)); err != nil {
			return fmt.Errorf("encode text for %q: %w", n.Tag, err)
		}
	}
	for _, child := range n.Children {
		if err := encodeNode(enc, child); err != nil {
			return err
		}
	}
	if err := enc.EncodeToken(xml.EndElement{Name: xml.Name{Local: n.Tag}}); err != nil {
		return fmt.Errorf("encode end element %q: %w", n.Tag, err)
	}
	return nil
}

// Parse reads XML into a Node tree (used for loading the base world SDF template).
func Parse(r io.Reader) (*Node, error) {
	dec := xml.NewDecoder(r)
	for {
		tok, err := dec.Token()
		if err != nil {
			return nil, fmt.Errorf("parse sdf: %w", err)
		}
		if se, ok := tok.(xml.StartElement); ok {
			return buildNode(dec, se)
		}
	}
}

func buildNode(dec *xml.Decoder, se xml.StartElement) (*Node, error) {
	n := &Node{Tag: se.Name.Local}
	for _, a := range se.Attr {
		n.Attrs = append(n.Attrs, [2]string{a.Name.Local, a.Value})
	}
	for {
		tok, err := dec.Token()
		if err != nil {
			return nil, fmt.Errorf("read sdf token: %w", err)
		}
		switch t := tok.(type) {
		case xml.StartElement:
			child, buildErr := buildNode(dec, t)
			if buildErr != nil {
				return nil, buildErr
			}
			n.Children = append(n.Children, child)
		case xml.CharData:
			if text := strings.TrimSpace(string(t)); text != "" {
				n.Text += text
			}
		case xml.EndElement:
			return n, nil
		}
	}
}
