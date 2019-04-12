%{
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Pranav Bhounsule
%}
function s = open_log(fname)

LOGGEROL_SEP = '/';

if ~exist(fname,'file')
  error('File not found: %s', fname)
end

fileID = fopen(fname);

line1 = fgetl(fileID);
if line1(1) == LOGGEROL_SEP
  % New style log
  newStyleLog = true;
  line1split = strsplit(line1(2:end), LOGGEROL_SEP);
  fprintf(1, 'New log style version %s\n', line1split{1});
  keys = strsplit(line1split{3},',');
  fmt = struct;%line1split{2};
  data = struct;
  for i=1:length(keys)
    [startIndex,endIndex] = regexp(keys{i}, '\d+');
    aa = keys{i};
    ndataincol = str2num(aa(startIndex:endIndex));
    data.(aa(1:startIndex-1)) = single(zeros(0, ndataincol));
    fmt.(aa(1:startIndex-1)) = aa(endIndex+1:end);
  end

  fprintf('Opened %s\nKeys: %s\n', fname, strjoin(fieldnames(data),','))
else
  % keep old
  newStyleLog = false;
  colnames = fgetl(fileID);
  fmt = fgetl(fileID);
  fprintf('Opened %s\nKeys: %s\nFormat: %s\n', fname, colnames, fmt)
  colnames = strsplit(colnames, ',');
  data = zeros([0,length(colnames)]);
end

i = 0;

try
  while ~feof(fileID)
    alignment1 = fread(fileID,1,'uint8');
    if alignment1 ~= 170
      continue;
    end
    alignment2 = fread(fileID,1,'uint8');
    if alignment2 ~= 187
      continue;
    end

    i = i+1;

    if newStyleLog
      fields = fieldnames(data);
      for j=1:numel(fields)
        colfmt = fmt.(fields{j});

        % fields{j}
        ndataincol = size(data.(fields{j}),2);
        if (colfmt=='f')
          type = 'float=>float';
        elseif (colfmt=='I')
          type = 'uint32=>float';
        elseif (colfmt =='B')
          type = 'uint8=>float';
        elseif (colfmt=='h')
          type = 'int16=>float';
        else
          error('unspecfied datatype in fmt');
        end
        newrow = fread(fileID,ndataincol,type)';
        if strcmp(fields{j},'eul') || strcmp(fields{j},'angVel') || strcmp(fields{j},'pos') || strcmp(fields{j},'torq') || strcmp(fields{j},'Vbat') || strcmp(fields{j},'xdot')
          newrow = 0.001 * newrow;
        elseif strcmp(fields{j},'acc') || strcmp(fields{j},'Ibat') 
          newrow = 0.01 * newrow;
        end
        data.(fields{j}) = [data.(fields{j}); newrow];
        % data
      end


    else
      % old same
      for j=1:length(fmt)
        if (fmt(j)=='f')
          type = 'float=>float';
        elseif (fmt(j)=='I')
          type = 'uint32=>float';
        elseif (fmt(j) =='B')
          type = 'uint8=>float';
        else
          error('unspecfied datatype in fmt');
        end

        data(i,j)=fread(fileID,1,type);
      end
      
    end
     
  end
  fclose(fileID);
catch
  disp('Finished reading');
end

if newStyleLog
  fprintf('Avg data rate = %.2f Hz\n',1000/mean(diff(data.t)))
  s = data;
else
  % Return data in a struct
  s = struct();
  for cn=1:length(colnames)
    % convert time to seconds
    if strcmp(colnames{cn},'t')
      data(:,cn) = 0.001 * data(:,cn);
    end
    s = setfield(s,colnames{cn},data(:,cn));
  end
end

end
