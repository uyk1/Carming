package com.carming.backend.place.dto.request;

import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.List;

@NoArgsConstructor
@Data
public class PlaceSearch {

    private final static Long DEFAULT_SIZE = 50L;

    private List<String> regions;

    private String category;

    private Long tagId;

    private Long size = DEFAULT_SIZE;

    public PlaceSearch(List<String> regions, String category, Long tagId, Long size) {
        this.regions = regions;
        this.category = category;
        this.tagId = tagId;
        this.size = size == null ? DEFAULT_SIZE : size;
    }
}
