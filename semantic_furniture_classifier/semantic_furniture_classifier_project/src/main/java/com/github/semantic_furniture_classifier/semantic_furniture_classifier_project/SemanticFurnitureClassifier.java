/*
 * Copyright (C) 2016 Martin Günther.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.semantic_furniture_classifier.semantic_furniture_classifier_project;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.mindswap.pellet.jena.PelletReasonerFactory;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import au.com.bytecode.opencsv.CSVReader;
import au.com.bytecode.opencsv.CSVWriter;

import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.ontology.OntClass;
import com.hp.hpl.jena.ontology.OntModel;
import com.hp.hpl.jena.ontology.OntResource;
import com.hp.hpl.jena.rdf.model.ModelFactory;
import com.hp.hpl.jena.rdf.model.Property;
import com.hp.hpl.jena.rdf.model.RDFNode;
import com.hp.hpl.jena.rdf.model.Resource;
import com.hp.hpl.jena.util.iterator.ExtendedIterator;

import semantic_object_maps_msgs.*;
import std_msgs.ColorRGBA;
import std_msgs.Header;
import visualization_msgs.Marker;
import visualization_msgs.MarkerArray;

public class SemanticFurnitureClassifier {
  private static final String OWL_FILE_NAME = "file:owl/furniture.owl";
  private static final String NS = "http://www.informatik.uni-osnabrueck.de/mguenthe/owl/furniture.owl#";

  // the maximum x (resp. z) difference for isAbove
  private static final double MAX_XY_DIFF = 0.5;

  private OntModel model = ModelFactory.createOntologyModel(PelletReasonerFactory.THE_SPEC);

  private final OntClass Plane;
  private final OntClass HorizontalPlane;
  private final OntClass VerticalPlane;

  private final OntClass Shelf;
  private final OntClass Table;
  private final OntClass Chair;

  private final Property hasAngle;

  private final Property hasPosX;
  private final Property hasPosY;
  private final Property hasPosZ;

  private final Property hasBboxX;
  private final Property hasBboxY;
  private final Property hasBboxZ;

  private final Property hasSize;
  private final Property isAbove;
  private final Property isBelow;
  private final Property isPerpendicular;

  private Header lastSeenHeader; // TODO: store as plane property

  public SemanticFurnitureClassifier() {
    model.read(OWL_FILE_NAME);

    Plane = model.getOntClass(NS + "Plane");
    HorizontalPlane = model.getOntClass(NS + "HorizontalPlane");
    VerticalPlane = model.getOntClass(NS + "VerticalPlane");

    Shelf = model.getOntClass(NS + "Shelf");
    Table = model.getOntClass(NS + "Table");
    Chair = model.getOntClass(NS + "Chair");

    hasAngle = model.getProperty(NS + "hasAngle");

    hasPosX = model.getProperty(NS + "hasPosX");
    hasPosY = model.getProperty(NS + "hasPosY");
    hasPosZ = model.getProperty(NS + "hasPosZ");

    hasBboxX = model.getProperty(NS + "hasBboxX");
    hasBboxY = model.getProperty(NS + "hasBboxY");
    hasBboxZ = model.getProperty(NS + "hasBboxZ");

    hasSize = model.getProperty(NS + "hasSize");
    isAbove = model.getProperty(NS + "isAbove");
    isBelow = model.getProperty(NS + "isBelow");
    isPerpendicular = model.getProperty(NS + "isPerpendicular");

    // load the model into the reasoner
    model.prepare();
  }

  public void processPatchArray(PlanarPatchArray patchArray) {
    List<Individual> horizontalPlanes = new ArrayList<Individual>();
    List<Individual> verticalPlanes = new ArrayList<Individual>();
    try {
      for (PlanarPatch patch : patchArray.getPatches()) {
        processPatch(patch, horizontalPlanes, verticalPlanes);
      }
      addAboveProperties(horizontalPlanes);
      addPerpendicularProperties(horizontalPlanes, verticalPlanes);
    } catch (InvalidInputDataException e) {
      e.printStackTrace();
    }
  }

  private void processPatch(PlanarPatch patch, List<Individual> horizontalPlanes,
      List<Individual> verticalPlanes) throws InvalidInputDataException {
    String id = "patch_" + patch.getId();

    byte orientation = patch.getOrientation();
    Individual newPlane;
    if (orientation == PlanarPatch.ORIENTATION_HORIZONTAL) {
      newPlane = model.createIndividual(NS + id, HorizontalPlane);
      horizontalPlanes.add(newPlane);
    } else if (orientation == PlanarPatch.ORIENTATION_VERTICAL) {
      newPlane = model.createIndividual(NS + id, VerticalPlane);
      verticalPlanes.add(newPlane);
    } else if (orientation == PlanarPatch.ORIENTATION_UNKNOWN) {
      newPlane = model.createIndividual(NS + id, Plane);
    } else {
      throw new InvalidInputDataException("ERROR: Unknown orientation \"" + orientation + "\"");
    }

    // unused fields:
    // patch.getContour();
    // patch.getNormal();

    lastSeenHeader = patch.getHeader();
    if (lastSeenHeader.getFrameId().equals("")) {
      lastSeenHeader.setFrameId("map");
    }

    // float angle = normalizeAngle(Float.parseFloat(nextLine[13]));

    // newPlane.addLiteral(hasAngle, angle);
    newPlane.addLiteral(hasPosX, (float) patch.getCentroid().getX());
    newPlane.addLiteral(hasPosY, (float) patch.getCentroid().getY());
    newPlane.addLiteral(hasPosZ, (float) patch.getCentroid().getZ());
    newPlane.addLiteral(hasBboxX, (float) patch.getBbox().getX());
    newPlane.addLiteral(hasBboxY, (float) patch.getBbox().getY());
    newPlane.addLiteral(hasBboxZ, (float) patch.getBbox().getZ());
    newPlane.addLiteral(hasSize, patch.getArea());
  }

  private void read(String inputFile, int filenum) throws IOException {
    System.out.println("Reading file #" + filenum);

    // for each line in the file, create a new individual and add it to the
    // model
    CSVReader reader = new CSVReader(new FileReader(inputFile), ' ');
    String[] nextLine;
    List<Individual> horizontalPlanes = new ArrayList<Individual>();
    List<Individual> verticalPlanes = new ArrayList<Individual>();
    try {
      while ((nextLine = reader.readNext()) != null) {
        processLine(nextLine, filenum, horizontalPlanes, verticalPlanes);
      }
      addAboveProperties(horizontalPlanes);
      addPerpendicularProperties(horizontalPlanes, verticalPlanes);
    } catch (InvalidInputDataException e) {
      e.printStackTrace();
    } finally {
      reader.close();
    }
  }

  private void processLine(String[] nextLine, int filenum, List<Individual> horizontalPlanes,
      List<Individual> verticalPlanes) throws InvalidInputDataException {
    if (nextLine.length != 14) {
      throw new InvalidInputDataException("Wrong number of fields in data file: " + nextLine.length);
    }
    String id = "plane_" + filenum + "_" + nextLine[0];

    float posX = Float.parseFloat(nextLine[1]);
    float posY = Float.parseFloat(nextLine[2]);
    float posZ = Float.parseFloat(nextLine[3]);

    // float normalX = Float.parseFloat(nextLine[4]);
    // float normalY = Float.parseFloat(nextLine[5]);
    // float normalZ = Float.parseFloat(nextLine[6]);

    float size;
    try {
      // Try size after hole closing first
      size = Float.parseFloat(nextLine[8]);
    } catch (NumberFormatException e) {
      try {
        // size after hole closing is "-nan", try size before hole
        // closing
        size = Float.parseFloat(nextLine[7]);
      } catch (NumberFormatException e2) {
        System.err.println("WARNING: Both size before and after hole closing are -nan for " + id
            + ", ignoring!");
        return;
      }
    }

    float bboxX = Float.parseFloat(nextLine[9]);
    float bboxY = Float.parseFloat(nextLine[10]);
    float bboxZ = Float.parseFloat(nextLine[11]);

    String orientation = nextLine[12];

    float angle = normalizeAngle(Float.parseFloat(nextLine[13]));

    Individual newPlane;
    if (orientation.equals("h")) {
      newPlane = model.createIndividual(NS + id, HorizontalPlane);
      horizontalPlanes.add(newPlane);
    } else if (orientation.equals("v")) {
      newPlane = model.createIndividual(NS + id, VerticalPlane);
      verticalPlanes.add(newPlane);
    } else if (orientation.equals("u")) {
      newPlane = model.createIndividual(NS + id, Plane);
    } else {
      throw new InvalidInputDataException("ERROR: Unknown orientation \"" + orientation + "\"");
    }

    newPlane.addLiteral(hasAngle, angle);
    newPlane.addLiteral(hasPosX, posX);
    newPlane.addLiteral(hasPosY, posY);
    newPlane.addLiteral(hasPosZ, posZ);
    newPlane.addLiteral(hasBboxX, bboxX);
    newPlane.addLiteral(hasBboxY, bboxY);
    newPlane.addLiteral(hasBboxZ, bboxZ);
    newPlane.addLiteral(hasSize, size);
  }

  private void write(String outputFile) throws IOException {
    CSVWriter writer = new CSVWriter(new FileWriter(outputFile), ' ', CSVWriter.NO_QUOTE_CHARACTER);

    List<OntClass> classes = new ArrayList<OntClass>(3);
    classes.add(Table);
    classes.add(Chair);
    classes.add(Shelf);

    for (OntClass ontClass : classes) {
      for (Iterator<?> i = ontClass.listInstances(); i.hasNext();) {
        String[] nextLine = new String[10];

        Individual ind = (Individual) i.next();

        // 0: id
        nextLine[0] = ind.getLocalName().substring(5);

        // 1: type
        nextLine[1] = ontClass.getLocalName();

        // 2, 3, 4: centroid xyz
        nextLine[2] = ind.getPropertyValue(hasPosX).asLiteral().getString();
        nextLine[3] = ind.getPropertyValue(hasPosY).asLiteral().getString();
        nextLine[4] = ind.getPropertyValue(hasPosZ).asLiteral().getString();

        // 5, 6, 7: bbox xyz
        nextLine[5] = ind.getPropertyValue(hasBboxX).asLiteral().getString();
        nextLine[6] = ind.getPropertyValue(hasBboxY).asLiteral().getString();
        nextLine[7] = ind.getPropertyValue(hasBboxZ).asLiteral().getString();

        // 8, 9: angle, centroid angle
        float angle = ind.getPropertyValue(hasAngle).asLiteral().getFloat();
        float centroidAngle = (float) 0.0;

        // handle special case chairs: centroid of seat, but angle of
        // backrest
        // (flipped towards seat)
        if (ind.hasOntClass(Chair)) {
          for (Iterator<?> it = ind.listPropertyValues(isPerpendicular); it.hasNext();) {
            RDFNode node = (RDFNode) it.next();

            if (node.canAs(Individual.class)) {
              Individual vertical = (Individual) node.as(Individual.class);

              // calculate angle between centroid of vertical and
              // centroid of
              // horizontal plane
              double x = ind.getPropertyValue(hasPosX).asLiteral().getDouble()
                  - vertical.getPropertyValue(hasPosX).asLiteral().getDouble();
              double y = ind.getPropertyValue(hasPosY).asLiteral().getDouble()
                  - vertical.getPropertyValue(hasPosY).asLiteral().getDouble();
              centroidAngle = (float) Math.toDegrees(Math.atan2(y, x));

              if (Math.abs(normalizeAngle(angle - centroidAngle)) > 90.0) {
                System.out.println("Chair normal angle: " + angle + ", centroid angle: "
                    + centroidAngle + " --- flipping normal angle to "
                    + normalizeAngle(angle + (float) 180.0));
                angle = normalizeAngle(angle + (float) 180.0);
              } else {
                System.out.println("Chair normal angle: " + angle + ", centroid angle: "
                    + centroidAngle + " --- not flipping");
              }
            } else {
              System.err.println(node + " cannot be converted to individual!");
              continue;
            }

            // we're only interested in the first individual for now
            break;
          }
        }

        nextLine[8] = Float.toString(angle);
        nextLine[9] = Float.toString(centroidAngle);

        writer.writeNext(nextLine);
      }
    }

    writer.close();
  }

  private void addAboveProperties(List<Individual> horizontalPlanes) {
    for (Individual upper : horizontalPlanes) {
      float upperX = upper.getPropertyValue(hasPosX).asLiteral().getFloat();
      float upperY = upper.getPropertyValue(hasPosY).asLiteral().getFloat();
      float upperZ = upper.getPropertyValue(hasPosZ).asLiteral().getFloat();

      for (Individual lower : horizontalPlanes) {
        float lowerZ = lower.getPropertyValue(hasPosZ).asLiteral().getFloat();
        float xDiff = Math.abs(upperX - lower.getPropertyValue(hasPosX).asLiteral().getFloat());
        float yDiff = Math.abs(upperY - lower.getPropertyValue(hasPosY).asLiteral().getFloat());

        if (upperZ > lowerZ && xDiff < MAX_XY_DIFF && yDiff < MAX_XY_DIFF)
          upper.addProperty(isAbove, lower);
      }
    }
  }

  private void addPerpendicularProperties(List<Individual> horizontalPlanes,
      List<Individual> verticalPlanes) {
    for (Individual horizontal : horizontalPlanes) {
      float horizontalX = horizontal.getPropertyValue(hasPosX).asLiteral().getFloat();
      float horizontalY = horizontal.getPropertyValue(hasPosY).asLiteral().getFloat();
      float horizontalZ = horizontal.getPropertyValue(hasPosZ).asLiteral().getFloat();

      double closestDist = Double.POSITIVE_INFINITY;
      Individual closestVertical = null;
      for (Individual vertical : verticalPlanes) {
        float verticalZ = vertical.getPropertyValue(hasPosZ).asLiteral().getFloat();
        float xDiff = Math.abs(horizontalX
            - vertical.getPropertyValue(hasPosX).asLiteral().getFloat());
        float yDiff = Math.abs(horizontalY
            - vertical.getPropertyValue(hasPosY).asLiteral().getFloat());
        double dist = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

        // find closest vertical plane above the horizontal plane
        if (verticalZ > horizontalZ && xDiff < MAX_XY_DIFF && yDiff < MAX_XY_DIFF
            && dist < closestDist) {
          closestDist = dist;
          closestVertical = vertical;
        }
      }
      if (closestVertical != null)
        horizontal.addProperty(isPerpendicular, closestVertical);
    }
  }

  public void printDebug() {
    // get all instances of Shelf class
    for (Iterator<?> i = Shelf.listInstances(); i.hasNext();) {
      Individual shelf = (Individual) i.next();

      // get the info about this specific individual
      String name = shelf.getLocalName();
      Resource type = shelf.getRDFType();

      // print the results
      System.out.println("SHELF");
      System.out.println("Name: " + name);
      System.out.println("Type: " + type.getLocalName());
      for (Iterator<?> j = shelf.listPropertyValues(isBelow); j.hasNext();) {
        OntResource plane = (OntResource) j.next();
        System.out.println("isBelow: " + plane.getLocalName());
      }
      System.out.println("Height: " + shelf.getPropertyValue(hasPosZ).asLiteral().getFloat());
      System.out.println("Size:   " + shelf.getPropertyValue(hasSize).asLiteral().getFloat());
      System.out.println();
    }

    // get all instances of Table class
    for (Iterator<?> i = Table.listInstances(); i.hasNext();) {
      Individual table = (Individual) i.next();

      // get the info about this specific individual
      String name = table.getLocalName();
      Resource type = table.getRDFType();

      // print the results
      System.out.println("TABLE");
      System.out.println("Name:   " + name);
      System.out.println("Type:   " + type.getLocalName());
      System.out.println("Height: " + table.getPropertyValue(hasPosZ).asLiteral().getFloat());
      System.out.println("Size:   " + table.getPropertyValue(hasSize).asLiteral().getFloat());
      System.out.println();
    }

    // get all instances of Chair class
    for (Iterator<?> i = Chair.listInstances(); i.hasNext();) {
      Individual chair = (Individual) i.next();

      // get the info about this specific individual
      String name = chair.getLocalName();
      Resource type = chair.getRDFType();

      // print the results
      System.out.println("CHAIR");
      System.out.println("Name:   " + name);
      System.out.println("Type:   " + type.getLocalName());
      System.out.println("Height: " + chair.getPropertyValue(hasPosZ).asLiteral().getFloat());
      System.out.println("Size:   " + chair.getPropertyValue(hasSize).asLiteral().getFloat());
      System.out.println();
    }
  }

  public MarkerArray makeMarkers() {
    NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfig.getTopicMessageFactory();

    List<Marker> markers = new ArrayList<Marker>();

    List<Individual> individuals = new ArrayList<Individual>();
    for (ExtendedIterator<? extends OntResource> i = Shelf.listInstances(); i.hasNext();) {
      individuals.add((Individual) i.next());
    }
    for (ExtendedIterator<? extends OntResource> i = Chair.listInstances(); i.hasNext();) {
      individuals.add((Individual) i.next());
    }
    for (ExtendedIterator<? extends OntResource> i = Table.listInstances(); i.hasNext();) {
      individuals.add((Individual) i.next());
    }

    for (Individual ind : individuals) {
      Marker marker = messageFactory.newFromType(Marker._TYPE);
      marker.setHeader(lastSeenHeader);
      marker.setAction(Marker.ADD);
      marker.setType(Marker.TEXT_VIEW_FACING);
      marker.setNs("owl_classified_planes");
      marker.setId(markers.size());
      Vector3 scale = messageFactory.newFromType(Vector3._TYPE);
      scale.setX(0.1f);
      scale.setY(0.1f);
      scale.setZ(0.1f);
      marker.setScale(scale);
      ColorRGBA color = messageFactory.newFromType(ColorRGBA._TYPE);
      color.setR(1.0f);
      color.setG(1.0f);
      color.setB(1.0f);
      color.setA(1.0f);
      marker.setColor(color);

      String name = ind.getLocalName();
      Resource type = ind.getRDFType();

      Pose pose = messageFactory.newFromType(Pose._TYPE);
      Quaternion q = messageFactory.newFromType(Quaternion._TYPE);
      q.setW(1.0);
      pose.setOrientation(q);
      Point position = messageFactory.newFromType(Point._TYPE);
      position.setX(ind.getPropertyValue(hasPosX).asLiteral().getFloat());
      position.setY(ind.getPropertyValue(hasPosY).asLiteral().getFloat());
      position.setZ(ind.getPropertyValue(hasPosZ).asLiteral().getFloat());
      pose.setPosition(position);
      marker.setPose(pose);

      marker.setText(name + " (" + type.getLocalName() + ")");
      markers.add(marker);
    }

    MarkerArray msg = messageFactory.newFromType(MarkerArray._TYPE);
    msg.setMarkers(markers);
    return msg;
  }

  // returns [-180, 180]
  private static float normalizeAngle(float a) {
    float result = (float) (a - 360.0 * Math.floor(a / 360.0));
    if (result > 180.0)
      result -= 360.0;
    return result;
  }

  /**
   * args[0]: input file args[1]: output file
   */
  public static void main(String[] args) throws Exception {
    if (args.length < 2) {
      System.err
          .println("Usage: semantic_furniture_classifier_project.sh <input file 1> ... <input file n> <output file>");
      return;
    }

    SemanticFurnitureClassifier app = new SemanticFurnitureClassifier();

    long start = System.currentTimeMillis();
    for (int i = 0; i < args.length - 1; i++) {
      app.read(args[i], i);
    }
    app.write(args[args.length - 1]);
    long stop = System.currentTimeMillis();
    System.out.println("Processing took " + (stop - start) + " ms");
    app.printDebug();
    System.out.println("Normal program end.");
  }
}
