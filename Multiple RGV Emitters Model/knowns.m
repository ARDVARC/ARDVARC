classdef knowns
    properties
        emitter_count (1,1) double
        pointing_vectors__ (:,3) double
        distances__ (:,:) double
        uas_ (1,3) double
    end

    methods(Static)
        function k = get_knowns_from(uas_, emitters__)
            arguments (Input)
                uas_ (1,3) double
                emitters__ (:,3) double
            end
            arguments (Output)
                k (1,1) knowns
            end

            k = knowns;
            k.emitter_count = size(emitters__,1);
            k.uas_ = uas_;
            full_pointing_vectors__ = emitters__ - uas_;
            norms_ = vecnorm(full_pointing_vectors__, 2, 2);
            k.pointing_vectors__ = full_pointing_vectors__ ./ norms_;
            
            k.distances__ = zeros(k.emitter_count);
            for i = 1:k.emitter_count
                for j = 1:k.emitter_count
                    k.distances__(i,j) = norm(emitters__(i,:)-emitters__(j,:));
                end
            end
        end
    end

    methods
        function [this, mean_noise] = shake(this, angle_deg)
            arguments (Input)
                this (1,1) knowns
                angle_deg (1,1) double
            end
            orthogonal_vectors_big___ = zeros(3, 3, this.emitter_count);
            orthogonal_vectors_big___(:,1,:) = [-this.pointing_vectors__(:,2) this.pointing_vectors__(:,1) zeros(this.emitter_count,1)]';
            random_rotations_about_pointing_vectors___ = axang2rotm([this.pointing_vectors__ rand(this.emitter_count, 1)*2*pi]);
            random_orthogonal_vectors_big___ = pagemtimes(random_rotations_about_pointing_vectors___, orthogonal_vectors_big___);
            random_orthogonal_vectors__ = reshape(random_orthogonal_vectors_big___(:,1,:), 3, this.emitter_count)';
            mean_noise = rand(1)*deg2rad(angle_deg);
            noise = repmat(mean_noise, this.emitter_count, 1);
            axangs__ = [random_orthogonal_vectors__ noise];
            rotms___ = axang2rotm(axangs__);
            pointing_vectors_big___ = zeros(3, 3, this.emitter_count);
            pointing_vectors_big___(:,1,:) = this.pointing_vectors__';
            rotated_pointing_vectors_big___ = pagemtimes(rotms___, pointing_vectors_big___);
            this.pointing_vectors__ = reshape(rotated_pointing_vectors_big___(:,1,:), 3, this.emitter_count)';
        end
    end
end