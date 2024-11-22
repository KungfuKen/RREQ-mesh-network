clc
close all
clear all 

%define nodes
num_nodes = 10;

%nodes positions in space (200x50)
node_positions = rand(num_nodes, 2); 

node_positions(:, 2) = node_positions(:, 2) * 1000; %y coordinates 0-200
node_positions(:, 1) = node_positions(:, 1) * 500; %x coordinates 0-50

%display node positions
disp('Node Positions (x,y):');
disp(node_positions);


%connectivity matrix, 0 or 1 values
connectivity_matrix = randi([0,1], num_nodes, num_nodes);

%making connection matrix upper triangular to avoid duplicate connections
connectivity_matrix = triu(connectivity_matrix, 1);

%making matrix symmetrical
connectivity_matrix = connectivity_matrix + connectivity_matrix';

%ensure each node has connection >= 1
for i = 1:num_nodes
    if ~any(connectivity_matrix(i, :))
        randomNode = randi([1, num_nodes]);  % Pick a random node
        while randomNode == i
            randomNode = randi([1, num_nodes]);  % Ensure it's not the same node
        end
        connectivity_matrix(i, randomNode) = 1;
        connectivity_matrix(randomNode, i) = 1;  % Symmetrize
    end
end

% Display connectivity matrix
disp('Connectivity Matrix:');
disp(connectivity_matrix);

% Visualize the network
figure;

% Plot the nodes
scatter(node_positions(:, 1), node_positions(:, 2), 100, 'filled', 'b');
hold on;

% Add labels to the nodes
for i = 1:num_nodes
    text(node_positions(i, 1), node_positions(i, 2), [' ' num2str(i)], 'FontSize', 10);
end

% Plot the connections based on the connectivity matrix
for i = 1:num_nodes
    for j = i+1:num_nodes
        if connectivity_matrix(i, j) == 1
            % Draw a line between connected nodes
            plot([node_positions(i, 1), node_positions(j, 1)], ...
                 [node_positions(i, 2), node_positions(j, 2)], 'k-', 'LineWidth', 1.5);
        end
    end
end

% Set axis limits
xlim([0 1000]);
ylim([0 1000]);
xlabel('X Position');
ylabel('Y Position');
title('Mesh Network Visualization');
hold off;

%Routing implementation AODV protocal
%Initialize node states 
node_states = cell(num_nodes, 1); %Cell for routing tables

for i = 1:num_nodes
    node_states{i}.id = i;
    node_states{i}.routing_table = [];
end 

%Example: set src and dst nodes
source_node = 1
destination_node = num_nodes;

%Broadcast Route Request RREQ from source_node
visited_nodes = []; %Track visited nodes 
route_path = []; %Track path from source to destination 

%Recursive function to find route
function found = broadcast_rreq(current_node, destination_node, route_path)
    global connectivity_matrix visited_nodes; 

    %Mark the nodes as visited 
    visited_nodes = [visited_nodes, current_node];
    route_path = [route_path, current_node];

   %Check if current node is the destination 
   if current_node == destination_node
       disp('Route found:');
       disp(route_path);
       found = true;
       return;
   end 

   %Broadcast to all neighbors
   neighbors = find(connectivity_matrix(current_node, :) == 1)
   for neighbor = neighbors
       if ~ismember(neighbor, visited_nodes)
           found = broadcast_rreq(neighbor, destination_node, route_path);
           if found
               return;
           end 
       end 
   end 

   found = false; %No route found 
end 

%Start route discovery 
global connectivity_matrix visited_nodes;
broadcast_rreq(source_node, destination_node, []);

% Initialize routing tables for each node
for i = 1:num_nodes
    % Create an empty structure with predefined fields
    node_states{i} = struct('destination', [], 'next_hop', [], 'distance', []);
end

% Populate routing tables with example data
for i = 1:num_nodes
    % Example: Add routes to random nodes
    destinations = randperm(num_nodes, randi([1, num_nodes-1])); % Random destinations
    next_hops = destinations; % Assume direct neighbors for simplicity
    distances = randi([1, 10], size(destinations)); % Random distances
    
    % Assign data to the fields
    node_states{i}.destination = destinations;
    node_states{i}.next_hop = next_hops;
    node_states{i}.distance = distances;
end

% Visualize the network with routing tables and router numbers
figure;

% Plot the nodes
scatter(node_positions(:, 1), node_positions(:, 2), 100, 'filled', 'b');
hold on;

% Plot the connections based on the connectivity matrix
for i = 1:num_nodes
    for j = i+1:num_nodes
        if connectivity_matrix(i, j) == 1
            % Draw a line between connected nodes
            plot([node_positions(i, 1), node_positions(j, 1)], ...
                 [node_positions(i, 2), node_positions(j, 2)], 'k-', 'LineWidth', 1.5);
        end
    end
end

% Add routing table information to each node
for i = 1:num_nodes
    % Get the routing table for the current node
    routing_table = node_states{i};
    
    % Format the routing table for display
    if isempty(routing_table)
        rt_display = sprintf('Node %d\nEmpty Table', i);
    else
        % Add the router number at the top
        rt_display = sprintf('Node %d\nDest: [%s]\nNextHop: [%s]\nDist: [%s]', ...
            i, ...
            num2str(routing_table.destination), ...
            num2str(routing_table.next_hop), ...
            num2str(routing_table.distance));
    end

    % Add the routing table as a text annotation near the node
    text(node_positions(i, 1) + 2, node_positions(i, 2) + 2, rt_display, ...
        'FontSize', 8, 'BackgroundColor', 'white', 'EdgeColor', 'black');
end

% Set axis limits
xlim([0 500]);
ylim([0 1000]);
xlabel('X Position');
ylabel('Y Position');
title('Mesh Network with Routing Tables and Node Numbers');
hold off;

% Set up the figure
figure;
scatter(node_positions(:, 1), node_positions(:, 2), 100, 'filled', 'b');
hold on;

% Plot the connections
for i = 1:num_nodes
    for j = i+1:num_nodes
        if connectivity_matrix(i, j) == 1
            plot([node_positions(i, 1), node_positions(j, 1)], ...
                 [node_positions(i, 2), node_positions(j, 2)], 'k-', 'LineWidth', 1.5);
        end
    end
end

% Initialize animation variables
source_node = 1; % Starting node for RREQ
destination_node = num_nodes; % Target node
visited_nodes = false(1, num_nodes); % Keep track of visited nodes
visited_nodes(source_node) = true;

% Display RREQ broadcast
current_nodes = [source_node];
while ~visited_nodes(destination_node)
    new_nodes = [];
    for i = current_nodes
        neighbors = find(connectivity_matrix(i, :) & ~visited_nodes);
        for neighbor = neighbors
            % Animate the packet
            line([node_positions(i, 1), node_positions(neighbor, 1)], ...
                 [node_positions(i, 2), node_positions(neighbor, 2)], ...
                 'Color', 'r', 'LineWidth', 2);
            pause(2); % Pause to simulate movement
            visited_nodes(neighbor) = true;
            new_nodes = [new_nodes, neighbor];
        end
    end
    current_nodes = new_nodes; % Update current nodes
end

% Highlight destination node
scatter(node_positions(destination_node, 1), node_positions(destination_node, 2), ...
        150, 'filled', 'g'); % Mark destination node in green
title('RREQ Packet Propagation');
hold off;
