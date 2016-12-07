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

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import visualization_msgs.MarkerArray;

public class SemanticFurnitureClassifierNode extends AbstractNodeMain {
  SemanticFurnitureClassifier classifier;
  
  public SemanticFurnitureClassifierNode() {
    classifier = new SemanticFurnitureClassifier();
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("semantic_furniture_classifier_node");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    
    final Publisher<visualization_msgs.MarkerArray> markerPub = connectedNode.newPublisher(
        "owl_classified_planes", visualization_msgs.MarkerArray._TYPE);
    Subscriber<semantic_object_maps_msgs.PlanarPatchArray> subscriber = connectedNode
        .newSubscriber("lvr_classified_planes", semantic_object_maps_msgs.PlanarPatchArray._TYPE);
    subscriber
        .addMessageListener(new MessageListener<semantic_object_maps_msgs.PlanarPatchArray>() {
          @Override
          public void onNewMessage(semantic_object_maps_msgs.PlanarPatchArray msg) {
            log.info("semantic_furniture_classifier: got " + msg.getPatches().size() + " patches");
            classifier.processPatchArray(msg);
            classifier.printDebug();
            MarkerArray markerArray = classifier.makeMarkers();
            markerPub.publish(markerArray);
          }
        });
  }
}
