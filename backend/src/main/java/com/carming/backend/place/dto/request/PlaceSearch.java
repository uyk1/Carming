package com.carming.backend.place.dto.request;

import com.carming.backend.place.domain.PlaceCategory;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.List;

@NoArgsConstructor
@Data
public class PlaceSearch {

    private List<String> regions;

    private PlaceCategory category;


}
